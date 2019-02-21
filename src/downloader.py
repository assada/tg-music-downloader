# coding=utf-8
from __future__ import unicode_literals

import sys

import os
import shutil

import requests
import telegram
import youtube_dl
from PIL import Image
from mutagen.id3 import ID3, TPE1, TRCK, TALB, APIC, ID3NoHeaderError
from telegram import ChatAction
from telegram.ext import run_async

from src.helper import Helper

reload(sys)
sys.setdefaultencoding('utf8')


class Thumb:
    def __init__(self, cover):
        self.cover = cover
        self.cover_name = cover.name + '_'

    def __enter__(self):
        size = 90, 90
        im = Image.open(self.cover)
        im.thumbnail(size, Image.ANTIALIAS)
        im.save(self.cover_name, "JPEG")

        return open(self.cover_name)

    def __exit__(self, exp_type, exp_value, exp_tr):
        os.remove(self.cover_name)


class Tag:
    def __init__(self, mp3, info):
        self.title = info['title']
        self.url = info['thumbnail']
        self.mp3 = mp3

    def __enter__(self):
        try:
            audio = ID3(self.mp3)
        except ID3NoHeaderError:
            audio = ID3()

        title = self.title
        if ' – ' in self.title:
            data = self.title.split(' – ', 1)
        else:
            data = self.title.split(' - ', 1)
        performer = 'Unknown Artist'
        if len(data) >= 2:
            performer = data[0]
            title = data[1]

        audio['TPE1'] = TPE1(encoding=3, text=performer)
        audio['TIT2'] = TALB(encoding=3, text=title)
        audio['TRCK'] = TRCK(encoding=3, text=title)
        self.cover = self._get_cover(self.url)
        audio['APIC'] = APIC(
            encoding=3,
            mime='image/jpeg',
            type=3, desc=u'Cover',
            data=self.cover.read()
        )
        audio.save(self.mp3)

        return {'cover': self.cover, 'performer': performer, 'title': title, 'full_title': self.title}

    @staticmethod
    def _get_cover(url):
        file_name = Helper.formatting(url)
        response = requests.get(url, stream=True)
        with open(file_name, 'wb') as out_file:
            shutil.copyfileobj(response.raw, out_file)
        del response

        return open(file_name, 'rb')

    def __exit__(self, exp_type, exp_value, exp_tr):
        os.remove(self.cover.name)


class YoutubeDownloader:
    def __init__(self, config, path, link, download=True):
        self.path = path
        self.link = link
        self.download = download
        self.ydl_opts = {
            'outtmpl': path + '.%(ext)s',
            'format': 'bestaudio/best',
            'quiet': True,
            'nowarnings': True,
            'noplaylist': True,
            'nocheckcertificate': True,
            'extractaudio': True,
            'audioformat': 'mp3',
            'logger': config.get_logger(),
            'postprocessors': [
                {
                    'key': 'FFmpegExtractAudio',
                    'preferredcodec': 'mp3',
                    'preferredquality': config.quality
                }
            ],
        }

    def __enter__(self):
        with youtube_dl.YoutubeDL(self.ydl_opts) as ydl:
            path = '%s.mp3' % self.path
            if self.download is not False:
                self.download = not os.path.exists(path)

            return ydl.extract_info(self.link, download=self.download)

    def __exit__(self, exp_type, exp_value, exp_tr):
        return None


class Downloader:
    def __init__(self, config):
        self.config = config

    def get_fullname_by_tags(self, path):
        try:
            self.config.get_logger().info('Parsing tags for full name: %s', path)
            audio = ID3(path)
            return '%s - %s' % (audio['TPE1'], audio['TIT2'])
        except:
            self.config.get_logger().info('File %s does not have ID3 tags', path)
            return False

    @run_async
    def download_tg(self, bot, update, file_id):
        message = update.message.reply_text('🔍 Start downloading...', quote=True)
        self.config.get_logger().info('Processing file: %s', file_id)
        new_file = bot.get_file(file_id)
        message.edit_text(text='🧠 Downloading...')
        path = (self.config.destination + file_id + '.mp3').encode("utf-8")
        if new_file.download(path):
            fullname = self.get_fullname_by_tags(path)
            if fullname:
                new_path = '%s%s.mp3' % (self.config.destination, fullname.encode("utf-8"))
                os.rename(path, new_path)
                self.config.get_logger().info('Renamed %s...', new_path)
                path = new_path

            self.config.get_logger().info('Saved to: %s', path)
            message.edit_text(text='🎉 Saved!')
        else:
            message.edit_text(text='Error.')

    @run_async
    def question(self, bot, update, link):
        file_name = Helper.formatting(link)
        path = './temp/%s' % file_name
        message = update.message.reply_text('🔍 Searching...', quote=True)
        with YoutubeDownloader(self.config, path, link, False) as info:
            message.delete()
            button_list = [
                telegram.InlineKeyboardButton("Download", callback_data=link),
                telegram.InlineKeyboardButton("Cancel", callback_data='cancel')
            ]
            reply_markup = telegram.InlineKeyboardMarkup(Helper.build_menu(button_list, n_cols=2))
            update.message.reply_text(
                text='🚥 Download %s?' % info['title'],
                reply_markup=reply_markup,
                quote=True
            )

    @run_async
    def download_by_link(self, bot, update, link):
        bot.sendChatAction(chat_id=update.message.chat_id, action=ChatAction.TYPING)
        self.config.get_logger().info('🔍 Downloading by link: %s', link)
        file_name = Helper.formatting(link)
        path = './temp/%s' % file_name
        message = update.message.reply_text('🧠 Downloading...', quote=True)
        with YoutubeDownloader(self.config, path, link) as info:
            self.config.get_logger().info('Downloaded: %s', info['title'])
            path = '%s.mp3' % path
            too_big_message = None
            message.edit_text(text='🧠 Processing...')
            self.config.get_logger().info('Processing %s...', info['title'])

            with Tag(path, info) as tags:
                file_size = os.path.getsize(path) / (1024 * 1024)
                if not file_size > 50:
                    message.edit_text(text="🚀 Sending (%sMB)" % file_size)
                    bot.sendChatAction(chat_id=update.message.chat_id, action=ChatAction.UPLOAD_AUDIO)
                    with Thumb(tags['cover']) as thumb:
                        bot.sendAudio(
                            chat_id=update.message.chat_id,
                            audio=open(path, 'rb'),
                            title=tags['title'],
                            performer=tags['performer'],
                            thumb=thumb,
                            duration=info['duration']
                        )
                else:
                    self.config.get_logger().info('%s is too big for sending to telegram.', tags['full_title'])
                    too_big_message = \
                        message.reply_text(
                            'File is too big for sending to telegram.%s' %
                            (' But i will try save in storage...' if self.config.persist else ''),
                            quote=True
                        )
                if self.config.persist:
                    new_path = '%s%s.mp3' % (self.config.destination, tags['full_title'].encode("utf-8"))
                    self.config.get_logger().info('Moving to: %s', new_path)
                    message.edit_text(text="💾 Moving... (%sMB)" % file_size)
                    shutil.move(path, new_path)
                    message.edit_text(text="🎉 Saved.")
                else:
                    os.remove(path)
                    message.delete()
                if too_big_message is not None:
                    too_big_message.delete()
