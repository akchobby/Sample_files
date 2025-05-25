FROM python:3.10-slim

# Install Jupyter, ipykernel, and recommended tools
RUN pip install --no-cache-dir notebook jupyterlab ipykernel casadi

# 
RUN mkdir -p /root/workspace

# Set working directory
WORKDIR /workspace

