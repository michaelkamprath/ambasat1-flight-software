FROM python:3-alpine
LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="JSON-Formatted Telemetry Gathering Service"
LABEL org.label-schema.description="A simple web service for gathering JSON-formatted telemetry data. Allows data to be grouped into projects."
LABEL org.label-schema.usage="https://github.com/michaelkamprath/ambasat1-flight-software/blob/master/ground-software/telemetry-service/README.md"
LABEL org.label-schema.vcs-url="https://github.com/michaelkamprath/ambasat1-flight-software/blob/master/ground-software/telemetry-service/"
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