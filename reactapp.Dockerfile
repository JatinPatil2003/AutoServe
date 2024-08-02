FROM node:20-alpine

WORKDIR /app

COPY ./ReactApp/package*.json ./

RUN npm install

COPY ./ReactApp .

EXPOSE 3000

CMD ["npm", "start"]
