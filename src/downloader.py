# coding=utf-8
from __future__ import unicode_literals

import os
from mutagen.id3 import ID3, TPE1, TRCK, TALB, APIC, ID3NoHeaderError
from slugify import slugify
import youtube_dl
import requests
import shutil
from telegram.ext import run_async


class Downloader:
    def __init__(self, config):
        self.config = config

    def formatting(self, title):
        if ":" in title:
            title = title.replace(":", " -")
        if "?" in title:
            title = title.replace("?", "")
        if "/" in title:
            title = title.replace("/", "_")
        if "|" in title:
            title = title.replace("|", "_")
        if '"' in title:
            title = title.replace('"', "'")
        return slugify(title)

    @run_async
    def download_tg(self, bot, update, file_id):
        message = update.message.reply_text('Start downloading...', quote=True)
        self.config.getLogger().info('Processing file: %s', file_id)
        newFile = bot.get_file(file_id)
        message.edit_text(text='Downloading...')
        newFile.download(self.config.destination + '/' + file_id + '.mp3')
        message.edit_text(text='Done!')

    def add_tags(self, mp3, url, title, artist, track):
        urlName = self.formatting(url) + '.jpg'
        response = requests.get(url, stream=True)
        with open(urlName, 'wb') as out_file:
            shutil.copyfileobj(response.raw, out_file)
        del response

        try:
            audio = ID3(mp3)
        except ID3NoHeaderError:
            audio = ID3()

        cover = open(urlName, 'rb')

        audio['TPE1'] = TPE1(encoding=3, text=artist)
        audio['TIT2'] = TALB(encoding=3, text=title)
        audio['TRCK'] = TRCK(encoding=3, text=track)
        audio['APIC'] = APIC(
            encoding=3,
            mime='image/jpeg',
            type=3, desc=u'Cover',
            data=cover.read()
        )
        audio.save(mp3)

        del urlName
        return cover

    @run_async
    def download_by_link(self, bot, update, link):
        self.config.getLogger().info('Downloading by link: ' + link)
        fileName = self.formatting(link)
        path = self.config.destination + fileName + '.mp3'
        ydl_opts = {
            'outtmpl': path,
            'format': 'bestaudio/best',
            'postprocessors': [
                {
                    'key': 'FFmpegExtractAudio',
                    'preferredcodec': 'mp3',
                    'preferredquality': '320'
                }
            ],
        }
        message = update.message.reply_text('Start downloading...', quote=True)
        with youtube_dl.YoutubeDL(ydl_opts) as ydl:
            message.edit_text(text='Downloading...')
            info = ydl.extract_info(link, download=not os.path.exists(path))
            title = info['title']
            duration = info['duration']
            self.config.getLogger().info('Downloaded: ' + title)
        fullTitle = title
        if ' – ' in title:
            data = title.split(' – ', 1)
        else:
            data = title.split(' - ', 1)
        performer = 'Unknown Artist'
        if len(data) >= 2:
            performer = data[0]
            title = data[1]
        if os.path.exists(path):
            message.edit_text(text='Processing...')
            with self.add_tags(path, info['thumbnail'], title, performer, title) as cover:
                fileSize = os.path.getsize(path) / (1024 * 1024)
                message.edit_text(text="Sending (%sMB)" % fileSize)
                if not fileSize > 50:
                    bot.sendAudio(chat_id=update.message.chat_id, audio=open(path, 'rb'), title=title,
                                  performer=performer, thumb=cover, duration=duration)
                else:
                    message.edit_text('File is too big for sending to telegram. But i will try save in storage...')
                message.edit_text(text='Renaming...')
                os.rename(path, self.config.destination + fullTitle + '.mp3')
                message.edit_text(text='Done!')
        else:
            message.edit_text(text='Error downloading.')
