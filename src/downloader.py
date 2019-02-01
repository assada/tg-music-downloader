# coding=utf-8
from __future__ import unicode_literals

import sys

import os
import shutil

import requests
import youtube_dl
from mutagen.id3 import ID3, TPE1, TRCK, TALB, APIC, ID3NoHeaderError
from slugify import slugify
from telegram import ChatAction
from telegram.ext import run_async

reload(sys)
sys.setdefaultencoding('utf8')


class Downloader:
    def __init__(self, config):
        self.config = config

    @staticmethod
    def formatting(title):
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

    @staticmethod
    def get_fullname_by_tags(path):
        try:
            audio = ID3(path)
            return '%s - %s' % (audio['TPE1'], audio['TIT2'])
        except:
            return False

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

        return cover

    @run_async
    def download_tg(self, bot, update, file_id):
        message = update.message.reply_text('Start downloading...', quote=True)
        self.config.get_logger().info('Processing file: %s', file_id)
        newFile = bot.get_file(file_id)
        message.edit_text(text='Downloading...')
        path = (self.config.destination + file_id + '.mp3').encode("utf-8")
        if newFile.download(path):
            fullname = self.get_fullname_by_tags(path)
            if fullname:
                newPath = '%s%s.mp3' % (self.config.destination, fullname.encode("utf-8"))
                os.rename(path, newPath)
                path = newPath

            self.config.get_logger().info('Saved to: %s', path)
            message.edit_text(text='Done!')
        else:
            message.edit_text(text='Error.')

    @run_async
    def download_by_link(self, bot, update, link):
        bot.sendChatAction(chat_id=update.message.chat_id, action=ChatAction.TYPING)
        self.config.get_logger().info('Downloading by link: ' + link)
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
            self.config.get_logger().info('Downloaded: ' + title)
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
            self.config.get_logger().info('Processing %s...', fullTitle)
            with self.add_tags(path, info['thumbnail'], title, performer, title) as cover:
                fileSize = os.path.getsize(path) / (1024 * 1024)
                message.edit_text(text="Sending (%sMB)" % fileSize)
                if not fileSize > 50:
                    bot.sendChatAction(chat_id=update.message.chat_id, action=ChatAction.UPLOAD_AUDIO)
                    bot.sendAudio(
                        chat_id=update.message.chat_id,
                        audio=open(path, 'rb'),
                        title=title,
                        performer=performer,
                        thumb=cover,
                        duration=duration
                    )
                    os.remove(cover.name)
                else:
                    self.config.get_logger().info('%s is too big for sending to telegram.', fullTitle)
                    message.edit_text('File is too big for sending to telegram. But i will try save in storage...')
                message.edit_text(text='Renaming...')
                self.config.get_logger().info('Renamed %s...', fullTitle)
                newPath = self.config.destination + fullTitle.encode("utf-8") + '.mp3'
                os.rename(path, newPath)
                self.config.get_logger().info('Saved to %s...', newPath)
                message.edit_text(text='Done!')
        else:
            self.config.get_logger().info('Error downloading %s...', fullTitle)
            message.edit_text(text='Error downloading.')
