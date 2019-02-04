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

    def get_fullname_by_tags(self, path):
        try:
            self.config.get_logger().info('Parsing tags for full name: %s', path)
            audio = ID3(path)
            return '%s - %s' % (audio['TPE1'], audio['TIT2'])
        except:
            self.config.get_logger().info('File %s does not have ID3 tags', path)
            return False

    def _get_cover(self, url):
        self.config.get_logger().info('Downloading cover %s...', url)
        file_name = self.formatting(url)
        response = requests.get(url, stream=True)
        with open(file_name, 'wb') as out_file:
            shutil.copyfileobj(response.raw, out_file)
        del response
        self.config.get_logger().info('Cover %s saved', url)

        return open(file_name, 'rb')

    def add_tags(self, mp3, url, title, artist, track):
        self.config.get_logger().info('Add tags: %s', mp3)
        try:
            audio = ID3(mp3)
        except ID3NoHeaderError:
            self.config.get_logger().info('No tags for %s. Using empty...', mp3)
            audio = ID3()

        audio['TPE1'] = TPE1(encoding=3, text=artist)
        audio['TIT2'] = TALB(encoding=3, text=title)
        audio['TRCK'] = TRCK(encoding=3, text=track)
        cover = self._get_cover(url)
        audio['APIC'] = APIC(
            encoding=3,
            mime='image/jpeg',
            type=3, desc=u'Cover',
            data=cover.read()
        )
        audio.save(mp3)
        self.config.get_logger().info('Saved tags (artist=%s, title=%s, track=%s) for: %s', artist, title, track, mp3)

        return cover

    @run_async
    def download_tg(self, bot, update, file_id):
        message = update.message.reply_text('Start downloading...', quote=True)
        self.config.get_logger().info('Processing file: %s', file_id)
        new_file = bot.get_file(file_id)
        message.edit_text(text='Downloading...')
        path = (self.config.destination + file_id + '.mp3').encode("utf-8")
        if new_file.download(path):
            fullname = self.get_fullname_by_tags(path)
            if fullname:
                new_path = '%s%s.mp3' % (self.config.destination, fullname.encode("utf-8"))
                os.rename(path, new_path)
                self.config.get_logger().info('Renamed %s...', new_path)
                path = new_path

            self.config.get_logger().info('Saved to: %s', path)
            message.edit_text(text='Done!')
        else:
            message.edit_text(text='Error.')

    @run_async
    def download_by_link(self, bot, update, link):
        bot.sendChatAction(chat_id=update.message.chat_id, action=ChatAction.TYPING)
        self.config.get_logger().info('Downloading by link: ' + link)
        file_name = self.formatting(link)
        path = self.config.destination + file_name + '.mp3'
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
        full_title = title
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
            self.config.get_logger().info('Processing %s...', full_title)
            with self.add_tags(path, info['thumbnail'], title, performer, title) as cover:
                file_size = os.path.getsize(path) / (1024 * 1024)
                if not file_size > 50:
                    message.edit_text(text="Sending (%sMB)" % file_size)
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
                    self.config.get_logger().info('%s is too big for sending to telegram.', full_title)
                    message.edit_text('File is too big for sending to telegram. But i will try save in storage...')
                if self.config.persist:
                    message.edit_text(text='Renaming...')
                    self.config.get_logger().info('Renamed %s...', full_title)
                    new_path = self.config.destination + full_title.encode("utf-8") + '.mp3'
                    os.rename(path, new_path)
                    self.config.get_logger().info('Saved to %s...', new_path)
                else:
                    os.remove(path)
                message.edit_text(text='Done!')
        else:
            self.config.get_logger().info('Error downloading %s...', full_title)
            message.edit_text(text='Error downloading.')
