FROM python:3.10.12-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --upgrade pip && pip install -r requirements.txt
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN apt-get update && apt-get install -y \
    libx11-dev \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libtk8.6 \
    python3-pyqt5 \
    libglib2.0-0 \
    ffmpeg \
    tk-dev \
    && rm -rf /var/lib/apt/lists/*

COPY src /app/src
# ENTRYPOINT ["python", "/app/src/robot.py"]
# ENTRYPOINT ["/entrypoint.sh"]
# ENTRYPOINT ["python", "-c", "import matplotlib; print(matplotlib.get_backend())"]

