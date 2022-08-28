FROM python:3.10

RUN apt-get update && apt-get upgrade -y
RUN pip3 install --upgrade pip setuptools

# scipy
RUN apt-get install -y libopenblas-dev cmake gfortran
RUN pip3 install scipy

# OpenCV
RUN apt-get install -y libgl1-mesa-glx
RUN pip3 install opencv-python

# Flask
RUN pip3 install Flask

# DB
RUN apt-get install -y python3-dev default-libmysqlclient-dev
RUN pip3 install mysqlclient

RUN apt-get clean

CMD ["python3", "routes.py"]
