# Simple build
#   docker build -t flask-text-app .
# If we have a proxy; note that the --build_arg is optional
#   docker build --build-arg HTTPS_PROXY=$env:HTTPS_PROXY -t flask-text-app .
# Run
#   docker run -p 5000:5000 flask-text-app
# If we want to override the value of the HTTP_PROXY
#   docker run -e HTTPS_PROXY=$env:HTTPS_PROXY -p 5000:5000 flask-text-app

# Use an official Python runtime as a parent image
FROM python:3.9-slim

# Allow build-time arguments for proxies
ARG HTTPS_PROXY

# Set the build-time proxy arguments as environment variables
ENV HTTPS_PROXY=${HTTPS_PROXY}

# Set the working directory in the container
WORKDIR /app

# Copy the current directory contents into the container at /app
COPY . /app

# Install any needed packages specified in requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# Make port 5000 available to the world outside this container
EXPOSE 5000

# Define environment variable
ENV FLASK_APP=app.py
ENV FLASK_RUN_HOST=0.0.0.0

# Run app.py when the container launches
CMD ["flask", "run"]
