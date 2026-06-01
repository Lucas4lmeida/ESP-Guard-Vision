import os
import time
from datetime import datetime
from typing import List, Optional

from fastapi import FastAPI, File, UploadFile, Form, HTTPException, Request
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
import uvicorn
from sqlalchemy import create_engine, Column, Integer, String, Float, DateTime
from sqlalchemy.orm import declarative_base, sessionmaker, Session
from pydantic import BaseModel

# ---------- Configuração ----------
DATABASE_URL = "sqlite:///./guard.db"
UPLOAD_DIR = "uploads"
os.makedirs(UPLOAD_DIR, exist_ok=True)

# ---------- Banco de dados ----------
engine = create_engine(DATABASE_URL, connect_args={"check_same_thread": False})
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

class Detection(Base):
    __tablename__ = "detections"
    id = Column(Integer, primary_key=True, index=True)
    device_id = Column(String(50), index=True)
    timestamp = Column(DateTime, default=datetime.utcnow)
    filename = Column(String(200))
    score = Column(Float, default=0.0)
    box_x = Column(Integer, nullable=True)
    box_y = Column(Integer, nullable=True)
    box_w = Column(Integer, nullable=True)
    box_h = Column(Integer, nullable=True)

Base.metadata.create_all(bind=engine)

# ---------- FastAPI App ----------
app = FastAPI(title="ESP Guard Vision Backend", version="1.0")

# Servir arquivos estáticos (imagens enviadas)
app.mount("/uploads", StaticFiles(directory=UPLOAD_DIR), name="uploads")
templates = Jinja2Templates(directory="templates")

# ---------- Modelos Pydantic ----------
class DetectionOut(BaseModel):
    id: int
    device_id: str
    timestamp: datetime
    filename: str
    score: float
    box: Optional[str] = None

    class Config:
        orm_mode = True

class DeviceStatus(BaseModel):
    device_id: str
    last_seen: Optional[datetime]
    last_score: Optional[float]

# ---------- Rotas ----------
@app.post("/upload", response_model=DetectionOut)
async def upload_image(
    file: UploadFile = File(...),
    device_id: str = Form(...),
    score: float = Form(0.0),
    box_x: int = Form(None),
    box_y: int = Form(None),
    box_w: int = Form(None),
    box_h: int = Form(None),
):
    """Recebe uma imagem JPEG do ESP32, salva em disco e registra no banco."""
    if not file.content_type or not file.content_type.startswith("image/"):
        raise HTTPException(status_code=400, detail="Tipo de arquivo inválido")

    # Gera nome único
    timestamp = datetime.utcnow()
    ext = os.path.splitext(file.filename)[1] or ".jpg"
    safe_name = f"{device_id}_{int(time.time()*1000)}{ext}"
    file_path = os.path.join(UPLOAD_DIR, safe_name)

    # Salva arquivo
    with open(file_path, "wb") as buffer:
        buffer.write(await file.read())

    # Registra no banco
    db = SessionLocal()
    detection = Detection(
        device_id=device_id,
        timestamp=timestamp,
        filename=safe_name,
        score=score,
        box_x=box_x,
        box_y=box_y,
        box_w=box_w,
        box_h=box_h,
    )
    db.add(detection)
    db.commit()
    db.refresh(detection)
    db.close()
    return detection

@app.get("/images", response_model=List[DetectionOut])
async def list_images(device_id: Optional[str] = None, limit: int = 50):
    """Retorna metadados das últimas detecções."""
    db = SessionLocal()
    query = db.query(Detection).order_by(Detection.timestamp.desc())
    if device_id:
        query = query.filter(Detection.device_id == device_id)
    detections = query.limit(limit).all()
    db.close()

    result = []
    for d in detections:
        box = None
        if d.box_x is not None:
            box = f"{d.box_x},{d.box_y},{d.box_w},{d.box_h}"
        result.append(DetectionOut(
            id=d.id,
            device_id=d.device_id,
            timestamp=d.timestamp,
            filename=d.filename,
            score=d.score,
            box=box,
        ))
    return result

@app.get("/images/{image_id}")
async def get_image(image_id: int):
    """Retorna o arquivo de imagem correspondente ao ID da detecção."""
    db = SessionLocal()
    detection = db.query(Detection).filter(Detection.id == image_id).first()
    db.close()
    if not detection:
        raise HTTPException(status_code=404, detail="Detecção não encontrada")
    file_path = os.path.join(UPLOAD_DIR, detection.filename)
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail="Arquivo de imagem não encontrado")
    return FileResponse(file_path, media_type="image/jpeg")

@app.get("/status", response_model=List[DeviceStatus])
async def device_status():
    """Retorna o último estado conhecido de cada dispositivo."""
    db = SessionLocal()
    from sqlalchemy import func
    subq = db.query(
        Detection.device_id, func.max(Detection.timestamp).label("max_ts")
    ).group_by(Detection.device_id).subquery()
    last_detections = db.query(Detection).join(
        subq, (Detection.device_id == subq.c.device_id) & (Detection.timestamp == subq.c.max_ts)
    ).all()
    db.close()
    return [
        DeviceStatus(device_id=d.device_id, last_seen=d.timestamp, last_score=d.score)
        for d in last_detections
    ]

@app.get("/dashboard", response_class=HTMLResponse)
async def dashboard(request: Request):
    """Página HTML para visualizar as últimas detecções."""
    db = SessionLocal()
    detections = db.query(Detection).order_by(Detection.timestamp.desc()).limit(20).all()
    db.close()
    return templates.TemplateResponse("dashboard.html", {"request": request, "detections": detections})

@app.get("/config")
async def get_config(device_id: str):
    """Endpoint de configuração que o ESP consulta periodicamente."""
    return {
        "detect": True,
        "min_score": 0.5,
        "sleep_ms": 2000,
        "debug": False,
    }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)