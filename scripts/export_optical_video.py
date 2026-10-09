"""Use a project-local encoder to export existing AVI test evidence as MP4."""
from pathlib import Path
import shutil
import subprocess


def export_video(out):
    out = Path(out)
    encoder = shutil.which("ffmpeg")
    if encoder is None:
        analysis_python = Path(__file__).resolve().parents[1] / "build/yaw-analysis-venv/bin/python"
        result = subprocess.run([str(analysis_python), "-c", "import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())"],
                                capture_output=True, text=True)
        if result.returncode:
            return False
        encoder = result.stdout.strip()
    subprocess.run([encoder, "-loglevel", "error", "-y", "-i", str(out / "mission.avi"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", str(out / "mission.mp4")], check=True)
    return True
