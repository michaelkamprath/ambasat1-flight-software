FROM python:3-alpine
LABEL maintainer="Michael Kamprath <https://github.com/michaelkamprath>"

#
#   To run this docker, the following bind is expected
#       dst=/run/telemetry - This directory will contain the telemetry data files
#
#   This docker is listening on port 8000. Be sure to map that port.
#

EXPOSE 8000
RUN pip install bottle
RUN mkdir -p /run/telemetry
COPY telemetry-service.py /telemetry-service.py

CMD ["python", "/telemetry-service.py"]