FROM nodered/node-red:5.0.7

USER 0
WORKDIR /usr/src/node-red
RUN npm install node-red-dashboard@3.6.6 --no-audit --no-fund
USER 1000
