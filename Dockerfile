FROM alpine:edge

ENV LANG C.UTF-8
ENV BOT_TOKEN ''
ENV BOT_ADMIN ''
ENV BOT_DESTINATION ''
ENV BOT_PERSISTENCE 1

COPY . /tg-music-downloader

#Install mopidy
RUN apk update && apk add --no-cache --virtual .build-deps \
 	build-base \
	git \
 && apk add --no-cache \
	    jq \
        py-pip \
        python \
	    python-dev \
	    libressl-dev \
        make \
        libffi \
        libffi-dev \
        ffmpeg \
        py-cffi \
        bash \
 && pip install -U pip \
 && pip install -r /tg-music-downloader/requirements.txt \
 && apk del .build-deps

WORKDIR /tg-music-downloader

CMD [ "make", "run" ]