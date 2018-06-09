#!/usr/bin/env python
#coding: utf-8
import subprocess
import rospy

from std_msgs.msg import *

# -- Global変数 -- #
Speech_word = String()
Speech_flag = Bool()

# /speech_word のsubscribeとflagのスイッチ
def speech_word_sub(msg):
    global Speech_word
    global Speech_flag

    Speech_word = msg.data
    Speech_flag = True

def open_jtalk():
    global Speech_word
    global Speech_flag

    rospy.loginfo(str(Speech_word))

    # コマンドを変数に格納して、最後にまとめて実行する
    open_jtalk = ['open_jtalk'] # これはそのまま

    mech = ['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic'] # 辞書？

    htsvoice = ['-m', '/usr/share/hts-voice/nitech-jp-atr503-m001/nitech_jp_atr503_m001.htsvoice'] #声質のデータ
    #htsvoice = ['-m', '/path/to/hoge.htsvoice']


    # オプションの設定（http://moblog.absgexp.net/openjtalk/)
    all_pass = ['-a', '0.5']
    post_filter = ['-b', '0.3']
    speed = ['-r', '1.0']

    out_wav = ['-ow', './open_jtalk.wav'] # 音声ファイルの保存場所の設定
    out_log = ['-ot', './open_jtalk.log'] # ログの保存場所の設定

    # 変数をまとめる
    cmd = open_jtalk+mech+htsvoice+speed+out_wav+out_log+all_pass+post_filter

    # subprocessで実行する
    c = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    c.stdin.write(Speech_word)
    c.stdin.close()
    c.wait()

    #再生
    aplay = ['aplay', '-q', './open_jtalk.wav']
    wr = subprocess.Popen(aplay)

    # 発話が終わったらflagをfalseにする
    Speech_flag = False


def main():
    global Speech_word
    global Speech_flag

    rospy.init_node("tts_open_jtalk", anonymous=True)

    sub_topic = rospy.Subscriber("/speech_word", String, speech_word_sub)

    while not rospy.is_shutdown():
        if Speech_flag == True:
            open_jtalk()
        else:
            pass

if __name__ == "__main__":
    main()
