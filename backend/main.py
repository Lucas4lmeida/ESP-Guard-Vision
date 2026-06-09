"""
ESP Guard Vision — Backend V2 (FastAPI + SQLite)

Reescrita do backend para casar com o dashboard que consome o namespace /api/*
sem quebrar o firmware nem o dashboard antigo. Princípios desta versão:

  * Contrato do FIRMWARE intacto: POST /upload e GET /config têm exatamente os
    mesmos campos/formato de antes. Nenhuma alteração de firmware é necessária.
  * Namespace /api/* implementado: /api/detections, /api/devices, /api/status,
    /api/logs e /api/alerts (eram os 404 do log).
  * Compatibilidade reversa: /images, /images/{id} e /status continuam existindo
    para o dashboard.html antigo.
  * Imagens órfãs (registro no banco sem arquivo em disco) são filtradas das
    listagens — fim dos 404 em /uploads/*.jpg.
  * Bounding box correto: guardamos frame_w/frame_h (default XGA 1024x768, que é
    o que o firmware captura) para o front escalar a caixa sem chutar resolução.
  * logs/alerts derivados do que o backend já recebe (tabela de eventos leve),
    sem depender de dados novos do firmware.

Dependências:  pip install fastapi "uvicorn[standard]" sqlalchemy python-multipart
Executar:      python main.py   (ou: uvicorn main:app --host 0.0.0.0 --port 8000)
"""

import os
import re
import time
import logging
from datetime import datetime, timezone, timedelta
from typing import List, Optional

from fastapi import FastAPI, File, UploadFile, Form, HTTPException, Request, Depends
from fastapi.responses import HTMLResponse, FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from sqlalchemy import (
    create_engine, Column, Integer, String, Float, DateTime, func, event, text,
)
from sqlalchemy.engine import Engine
from sqlalchemy.orm import declarative_base, sessionmaker, Session
from pydantic import BaseModel, ConfigDict

# ----------------------------------------------------------------------------
# Configuração
# ----------------------------------------------------------------------------
DATABASE_URL = "sqlite:///./guard.db"
UPLOAD_DIR = "uploads"
DASHBOARD_FILE = "templates/dashboard.html"          # servido em / e /dashboard se existir

JPEG_MAGIC = b"\xff\xd8\xff"
MAX_IMAGE_BYTES = 4 * 1024 * 1024          # 4 MB

# Resolução nativa de captura do firmware (FRAMESIZE_XGA). Usada como default
# quando o firmware não informa frame_w/frame_h — assim o front escala a box
# corretamente em vez de assumir VGA.
DEFAULT_FRAME_W = 1024
DEFAULT_FRAME_H = 768

# Um dispositivo é considerado online se foi visto dentro desta janela. O
# firmware consulta /config a cada ~30 s, então 90 s tolera 2 perdas.
DEVICE_ONLINE_WINDOW_S = 90

# Score a partir do qual a detecção vira um "alerta".
ALERT_SCORE = 0.85

# Config global devolvida ao firmware (Tier 2: tornar por-dispositivo).
DEVICE_CONFIG = {
    "detect": True,
    "min_score": 0.5,
    "sleep_ms": 2000,
    "debug": False,
}

os.makedirs(UPLOAD_DIR, exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
log = logging.getLogger("guard")

# ----------------------------------------------------------------------------
# Banco de dados
# ----------------------------------------------------------------------------
engine = create_engine(DATABASE_URL, connect_args={"check_same_thread": False})
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()


@event.listens_for(Engine, "connect")
def _set_sqlite_pragmas(dbapi_conn, _record):
    """WAL melhora concorrência leitura/escrita (uploads chegando enquanto o
    dashboard faz polling). foreign_keys ligado por boa prática."""
    cur = dbapi_conn.cursor()
    cur.execute("PRAGMA journal_mode=WAL;")
    cur.execute("PRAGMA synchronous=NORMAL;")
    cur.execute("PRAGMA foreign_keys=ON;")
    cur.close()


class Detection(Base):
    __tablename__ = "detections"
    id = Column(Integer, primary_key=True, index=True)
    device_id = Column(String(50), index=True)
    timestamp = Column(DateTime(timezone=True), default=lambda: datetime.now(timezone.utc), index=True)
    filename = Column(String(200))
    score = Column(Float, default=0.0)
    box_x = Column(Integer, nullable=True)
    box_y = Column(Integer, nullable=True)
    box_w = Column(Integer, nullable=True)
    box_h = Column(Integer, nullable=True)
    frame_w = Column(Integer, nullable=True)
    frame_h = Column(Integer, nullable=True)


class Device(Base):
    """Registro de presença. Atualizado em /upload e em /config — não exige
    nada novo do firmware, só usa o que já chega."""
    __tablename__ = "devices"
    device_id = Column(String(50), primary_key=True, index=True)
    first_seen = Column(DateTime(timezone=True), default=lambda: datetime.now(timezone.utc))
    last_seen = Column(DateTime(timezone=True), default=lambda: datetime.now(timezone.utc), index=True)
    last_ip = Column(String(45), nullable=True)
    last_score = Column(Float, nullable=True)
    last_detection_at = Column(DateTime(timezone=True), nullable=True)


class Event(Base):
    """Linha do tempo de eventos para /api/logs e /api/alerts. Gerada pelo
    próprio backend a partir dos uploads e do heartbeat."""
    __tablename__ = "events"
    id = Column(Integer, primary_key=True, index=True)
    timestamp = Column(DateTime(timezone=True), default=lambda: datetime.now(timezone.utc), index=True)
    level = Column(String(10), default="info", index=True)   # info | warning | alert
    device_id = Column(String(50), nullable=True, index=True)
    kind = Column(String(30))                                 # ex.: upload, alert, device_online
    message = Column(String(300))


Base.metadata.create_all(bind=engine)


# ----------------------------------------------------------------------------
# Dependência de sessão (com rollback em erro)
# ----------------------------------------------------------------------------
def get_db():
    db = SessionLocal()
    try:
        yield db
    except Exception:
        db.rollback()
        raise
    finally:
        db.close()


# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------
_DEV_ID_RE = re.compile(r"[^A-Za-z0-9_-]")


def sanitize_device_id(raw: str) -> str:
    """Remove qualquer coisa que não seja alfanumérica/-/_ (o firmware manda
    12 hex do MAC, mas nunca confie na entrada para montar nome de arquivo)."""
    clean = _DEV_ID_RE.sub("", (raw or "").strip())
    return clean[:50] or "unknown"


def log_event(db: Session, level: str, kind: str, message: str,
              device_id: Optional[str] = None) -> None:
    db.add(Event(level=level, kind=kind, message=message, device_id=device_id))
    db.commit()


def touch_device(db: Session, device_id: str, ip: Optional[str],
                 score: Optional[float] = None, is_detection: bool = False) -> None:
    """Atualiza o heartbeat do dispositivo. Gera evento 'device_online' apenas
    na transição offline->online (evita inundar os logs a cada polling)."""
    now = datetime.now(timezone.utc)
    dev = db.get(Device, device_id)
    came_online = False

    if dev is None:
        dev = Device(device_id=device_id, first_seen=now, last_seen=now, last_ip=ip)
        db.add(dev)
        came_online = True
    else:
        gap = (now - _aware(dev.last_seen)).total_seconds() if dev.last_seen else 1e9
        if gap > DEVICE_ONLINE_WINDOW_S:
            came_online = True
        dev.last_seen = now
        if ip:
            dev.last_ip = ip

    if score is not None:
        dev.last_score = score
    if is_detection:
        dev.last_detection_at = now

    db.commit()
    if came_online:
        log_event(db, "info", "device_online", f"Dispositivo {device_id} online", device_id)


def _aware(dt: Optional[datetime]) -> Optional[datetime]:
    """Garante datetime tz-aware (SQLite pode devolver naive)."""
    if dt is None:
        return None
    return dt if dt.tzinfo else dt.replace(tzinfo=timezone.utc)


def file_exists(filename: str) -> bool:
    return bool(filename) and os.path.exists(os.path.join(UPLOAD_DIR, filename))


def is_online(last_seen: Optional[datetime]) -> bool:
    ls = _aware(last_seen)
    if ls is None:
        return False
    return (datetime.now(timezone.utc) - ls).total_seconds() <= DEVICE_ONLINE_WINDOW_S


def box_str(d: Detection) -> Optional[str]:
    if d.box_x is None:
        return None
    return f"{d.box_x},{d.box_y},{d.box_w},{d.box_h}"


# ----------------------------------------------------------------------------
# Modelos Pydantic
# ----------------------------------------------------------------------------
class DetectionOut(BaseModel):
    id: int
    device_id: str
    timestamp: datetime
    filename: str
    image_url: str
    score: float
    box: Optional[str] = None          # "x,y,w,h" (compatível com o front antigo)
    box_x: Optional[int] = None
    box_y: Optional[int] = None
    box_w: Optional[int] = None
    box_h: Optional[int] = None
    frame_w: int = DEFAULT_FRAME_W     # resolução de referência da box
    frame_h: int = DEFAULT_FRAME_H
    model_config = ConfigDict(from_attributes=True)


class DeviceOut(BaseModel):
    device_id: str
    online: bool
    last_seen: Optional[datetime] = None
    last_ip: Optional[str] = None
    last_score: Optional[float] = None
    last_detection_at: Optional[datetime] = None
    detections_total: int = 0


class DeviceStatusLegacy(BaseModel):
    device_id: str
    last_seen: Optional[datetime] = None
    last_score: Optional[float] = None


class EventOut(BaseModel):
    id: int
    timestamp: datetime
    level: str
    device_id: Optional[str] = None
    kind: str
    message: str
    model_config = ConfigDict(from_attributes=True)


class StatusSummary(BaseModel):
    server_time: datetime
    total_all: int
    total_today: int
    high_confidence_today: int
    devices_total: int
    devices_online: int
    last_event: Optional[DetectionOut] = None


# ----------------------------------------------------------------------------
# App
# ----------------------------------------------------------------------------
app = FastAPI(title="ESP Guard Vision Backend", version="2.0")

# CORS liberado para dev (dashboard servido de outra origem/porta). Restrinja
# em produção.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# Servir os arquivos de imagem enviados.
app.mount("/uploads", StaticFiles(directory=UPLOAD_DIR), name="uploads")


def to_detection_out(d: Detection) -> DetectionOut:
    return DetectionOut(
        id=d.id,
        device_id=d.device_id,
        timestamp=_aware(d.timestamp),
        filename=d.filename,
        image_url=f"/uploads/{d.filename}",
        score=d.score or 0.0,
        box=box_str(d),
        box_x=d.box_x, box_y=d.box_y, box_w=d.box_w, box_h=d.box_h,
        frame_w=d.frame_w or DEFAULT_FRAME_W,
        frame_h=d.frame_h or DEFAULT_FRAME_H,
    )


# ======================= ENDPOINTS DO FIRMWARE (contrato fixo) ===============
@app.post("/upload", response_model=DetectionOut)
async def upload_image(
    request: Request,
    file: UploadFile = File(...),
    device_id: str = Form(...),
    score: float = Form(0.0),
    box_x: int = Form(None),
    box_y: int = Form(None),
    box_w: int = Form(None),
    box_h: int = Form(None),
    frame_w: int = Form(None),        # opcional; firmware atual não envia
    frame_h: int = Form(None),
    db: Session = Depends(get_db),
):
    """Recebe a imagem JPEG do ESP32, valida, grava de forma atômica e registra.
    Mantém exatamente os campos que o firmware envia."""
    dev = sanitize_device_id(device_id)
    ip = request.client.host if request.client else None

    if not file.content_type or not file.content_type.startswith("image/"):
        log_event(db, "warning", "upload_rejected", f"Content-Type inválido de {dev}", dev)
        raise HTTPException(status_code=415, detail="Tipo de arquivo inválido")

    data = await file.read()
    if not data:
        raise HTTPException(status_code=400, detail="Imagem vazia")
    if len(data) > MAX_IMAGE_BYTES:
        raise HTTPException(status_code=413, detail="Imagem maior que o limite")
    if not data.startswith(JPEG_MAGIC):
        log_event(db, "warning", "upload_rejected", f"JPEG inválido de {dev}", dev)
        raise HTTPException(status_code=415, detail="Conteúdo não é um JPEG válido")

    timestamp = datetime.now(timezone.utc)
    ext = os.path.splitext(file.filename or "")[1] or ".jpg"
    safe_name = f"{dev}_{int(time.time() * 1000)}{ext}"
    file_path = os.path.join(UPLOAD_DIR, safe_name)

    # Gravação atômica: escreve .tmp e renomeia.
    tmp_path = file_path + ".tmp"
    with open(tmp_path, "wb") as buffer:
        buffer.write(data)
    os.replace(tmp_path, file_path)

    detection = Detection(
        device_id=dev,
        timestamp=timestamp,
        filename=safe_name,
        score=score,
        box_x=box_x, box_y=box_y, box_w=box_w, box_h=box_h,
        frame_w=frame_w or DEFAULT_FRAME_W,
        frame_h=frame_h or DEFAULT_FRAME_H,
    )
    db.add(detection)
    db.commit()
    db.refresh(detection)

    touch_device(db, dev, ip, score=score, is_detection=True)

    level = "alert" if (score or 0) >= ALERT_SCORE else "info"
    log_event(db, level, "alert" if level == "alert" else "upload",
              f"Detecção {dev} score {score:.2f}", dev)

    log.info("Upload OK: %s score=%.2f (%d bytes)", safe_name, score, len(data))
    return to_detection_out(detection)


@app.get("/config")
async def get_config(device_id: str, request: Request, db: Session = Depends(get_db)):
    """Config que o ESP consulta periodicamente. Também serve de heartbeat."""
    dev = sanitize_device_id(device_id)
    ip = request.client.host if request.client else None
    touch_device(db, dev, ip)
    return DEVICE_CONFIG


# ======================= NAMESPACE /api/* (dashboard novo) ===================
@app.get("/api/detections", response_model=List[DetectionOut])
async def api_detections(
    limit: int = 50,
    device_id: Optional[str] = None,
    min_score: float = 0.0,
    db: Session = Depends(get_db),
):
    limit = max(1, min(limit, 200))
    q = db.query(Detection).order_by(Detection.timestamp.desc())
    if device_id:
        q = q.filter(Detection.device_id == sanitize_device_id(device_id))
    if min_score > 0:
        q = q.filter(Detection.score >= min_score)
    # Busca um pouco a mais e filtra órfãos (arquivo ausente) até atingir o limite.
    rows = q.limit(limit * 3).all()
    out = [to_detection_out(d) for d in rows if file_exists(d.filename)]
    return out[:limit]


@app.get("/api/devices", response_model=List[DeviceOut])
async def api_devices(db: Session = Depends(get_db)):
    counts = dict(
        db.query(Detection.device_id, func.count(Detection.id))
          .group_by(Detection.device_id).all()
    )
    devices = db.query(Device).order_by(Device.last_seen.desc()).all()
    return [
        DeviceOut(
            device_id=d.device_id,
            online=is_online(d.last_seen),
            last_seen=_aware(d.last_seen),
            last_ip=d.last_ip,
            last_score=d.last_score,
            last_detection_at=_aware(d.last_detection_at),
            detections_total=counts.get(d.device_id, 0),
        )
        for d in devices
    ]


@app.get("/api/status", response_model=StatusSummary)
async def api_status(db: Session = Depends(get_db)):
    now = datetime.now(timezone.utc)
    start_today = datetime(now.year, now.month, now.day, tzinfo=timezone.utc)

    total_all = db.query(func.count(Detection.id)).scalar() or 0
    total_today = db.query(func.count(Detection.id)).filter(
        Detection.timestamp >= start_today).scalar() or 0
    high_today = db.query(func.count(Detection.id)).filter(
        Detection.timestamp >= start_today, Detection.score >= ALERT_SCORE).scalar() or 0

    devices = db.query(Device).all()
    devices_online = sum(1 for d in devices if is_online(d.last_seen))

    last = db.query(Detection).order_by(Detection.timestamp.desc()).first()
    return StatusSummary(
        server_time=now,
        total_all=total_all,
        total_today=total_today,
        high_confidence_today=high_today,
        devices_total=len(devices),
        devices_online=devices_online,
        last_event=to_detection_out(last) if last else None,
    )


@app.get("/api/logs", response_model=List[EventOut])
async def api_logs(limit: int = 120, device_id: Optional[str] = None,
                   db: Session = Depends(get_db)):
    limit = max(1, min(limit, 500))
    q = db.query(Event).order_by(Event.timestamp.desc())
    if device_id:
        q = q.filter(Event.device_id == sanitize_device_id(device_id))
    return q.limit(limit).all()


@app.get("/api/alerts", response_model=List[EventOut])
async def api_alerts(limit: int = 50, db: Session = Depends(get_db)):
    limit = max(1, min(limit, 200))
    return (db.query(Event).filter(Event.level == "alert")
              .order_by(Event.timestamp.desc()).limit(limit).all())


@app.get("/api/detections/{image_id}")
async def api_detection_image(image_id: int, db: Session = Depends(get_db)):
    d = db.get(Detection, image_id)
    if not d:
        raise HTTPException(status_code=404, detail="Detecção não encontrada")
    path = os.path.join(UPLOAD_DIR, d.filename)
    if not os.path.exists(path):
        raise HTTPException(status_code=404, detail="Arquivo de imagem não encontrado")
    return FileResponse(path, media_type="image/jpeg")


# ======================= COMPATIBILIDADE (dashboard antigo) ==================
@app.get("/images", response_model=List[DetectionOut])
async def list_images(device_id: Optional[str] = None, limit: int = 50,
                      db: Session = Depends(get_db)):
    return await api_detections(limit=limit, device_id=device_id, db=db)


@app.get("/images/{image_id}")
async def get_image(image_id: int, db: Session = Depends(get_db)):
    return await api_detection_image(image_id, db=db)


@app.get("/status", response_model=List[DeviceStatusLegacy])
async def device_status_legacy(db: Session = Depends(get_db)):
    devices = db.query(Device).order_by(Device.last_seen.desc()).all()
    return [
        DeviceStatusLegacy(
            device_id=d.device_id,
            last_seen=_aware(d.last_seen),
            last_score=d.last_score,
        )
        for d in devices
    ]


# ======================= UTILITÁRIOS / MANUTENÇÃO ============================
@app.get("/health")
async def health(db: Session = Depends(get_db)):
    db.execute(text("SELECT 1"))  # ping no banco
    return {"status": "ok", "time": datetime.now(timezone.utc)}


@app.post("/reconcile")
async def reconcile(db: Session = Depends(get_db)):
    """Remove do banco as detecções cujo arquivo não existe mais em disco.
    Resolve de vez os 404 em /uploads/*.jpg causados por registros órfãos."""
    rows = db.query(Detection).all()
    removed = 0
    for d in rows:
        if not file_exists(d.filename):
            db.delete(d)
            removed += 1
    db.commit()
    log_event(db, "info", "reconcile", f"{removed} registro(s) órfão(s) removido(s)")
    return {"removed": removed, "remaining": len(rows) - removed}


@app.get("/dashboard", response_class=HTMLResponse)
async def dashboard():
    if os.path.exists(DASHBOARD_FILE):
        with open(DASHBOARD_FILE, "r", encoding="utf-8") as f:
            return HTMLResponse(f.read())
    return HTMLResponse("<h2>Coloque o dashboard.html ao lado de main.py.</h2>")


@app.get("/", response_class=HTMLResponse)
async def root():
    return await dashboard()


@app.on_event("startup")
async def on_startup():
    db = SessionLocal()
    try:
        log_event(db, "info", "server_start", "Backend iniciado")
    finally:
        db.close()
    log.info("ESP Guard Vision backend pronto.")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)