# Telegram downloader bot

This bot wil download audio files which you are send to him.

### Requirements
* python 2.7
* ffmpeg

### Running
1) Install dependencies:
```bash
make build
```
2) Run:
```bash
make run "<token>" "<admin_username>" "<destination_path>"
```

or using docker:
1) Build container:
```bash
docker build -t tg-bot:latest .
```
2) Run container:
```bash
docker run --rm -v $(pwd)/files:/tg-music-downloader/files -e BOT_TOKEN='<tg_bot_token>' -e BOT_ADMIN='<tg_username>' -e BOT_DESTINATION='./files' tg-bot
```

### License

The OctoberCMS platform is open-sourced software licensed under the [MIT license](https://opensource.org/licenses/MIT).

### Contributing

Before sending a Pull Request, be sure to review the [Contributing Guidelines](CONTRIBUTING.md) first.