#!/bin/sh
echo こんにちは。私の名前はソビットです。 | open_jtalk \
-x /var/lib/mecab/dic/open-jtalk/naist-jdic \
-m /usr/share/hts-voice/nitech-jp-atr503-m001/nitech_jp_atr503_m001.htsvoice \
-ow  ~/open_jtalk/open_jtalk.wav \
-ot ~/open_jtalk/log.log \
#aplay open_jtalk.wav |
#rm open_jtalk.wav
