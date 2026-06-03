"""Tratamento opcional de imagens no backend (drop-in).

Use em /upload, logo após validar o JPEG e ANTES de gravar:

    from image_cleanup import sanitize_jpeg
    data = sanitize_jpeg(data)

Controlado por variáveis de ambiente (tudo desligado por padrão):
  GUARD_CROP_BOTTOM=16     -> recorta N linhas do rodapé (rede de segurança;
                              o firmware já recorta, isto cobre imagens antigas)
  GUARD_AUTOCONTRAST=1     -> leve realce de contraste (estético)

Requer Pillow:  pip install pillow
Observação: alterar a evidência muda o "original". Para perícia, prefira o
recorte no firmware e mantenha o backend só como rede de segurança.
"""
import io
import os

_CROP = int(os.environ.get("GUARD_CROP_BOTTOM", "0"))
_AUTO = os.environ.get("GUARD_AUTOCONTRAST", "0") == "1"


def sanitize_jpeg(data: bytes) -> bytes:
    if _CROP <= 0 and not _AUTO:
        return data  # nada a fazer: passa direto, sem custo
    try:
        from PIL import Image, ImageOps
    except ImportError:
        return data  # Pillow ausente -> não trata, não quebra
    try:
        im = Image.open(io.BytesIO(data)).convert("RGB")
        w, h = im.size
        if _CROP > 0 and h > _CROP + 8:
            im = im.crop((0, 0, w, h - _CROP))
        if _AUTO:
            im = ImageOps.autocontrast(im, cutoff=1)
        out = io.BytesIO()
        im.save(out, format="JPEG", quality=90)
        return out.getvalue()
    except Exception:
        return data  # qualquer falha -> devolve o original intacto
