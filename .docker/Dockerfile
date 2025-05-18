FROM python:3.10.12-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --upgrade pip && pip install -r requirements.txt

RUN apt-get update && apt-get install -y \
    libtk8.6 \
    python3-pyqt5 \
    libglib2.0-0 \
    tk-dev \
    ffmpeg

COPY src /app/src
