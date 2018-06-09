#!/usr/bin/env python
#coding: utf-8
import subprocess

def open_jtalk(t):
	open_jtalk=['open_jtalk']
	mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
	htsvoice=['-m', '/usr/share/hts-voice/nitech-jp-atr503-m001/nitech_jp_atr503_m001.htsvoice']
	#htsvoice=['-m', '/home/ohishikouta/catkin_ws/src/open_jtalk/voice/mei/mei_angry.thsvoice']
	#htsvoice=['-m', '/home/ohishikouta/catkin_ws/src/open_jtalk/voice/mei/mei_bashful.htsvoice']
	#htsvoice=['-m', '/home/ohishikouta/catkin_ws/src/open_jtalk/voice/mei/mei_happy.htsvoice']
	#htsvoice=['-m', '/home/ohishikouta/catkin_ws/src/open_jtalk/voice/mei/mei_normal.htsvoice']
	#htsvoice=['-m', '/home/ohishikouta/catkin_ws/src/open_jtalk/voice/mei/mei_sad.htsvoice']
	#htsvoice=['-m', '/home/ohishikouta/catkin_ws/src/open_jtalk/voice/slt/cmu_us_arctic_slt.htsvoice']
	all_pass=['-a', '0.5']
	post_filter=['-b', '0.3']
	speed=['-r', '1.0']
	out_wav=['-ow', 'open_jtalk.wav']
	out_log=['-ot', 'open_jtalk.log']
	cmd=open_jtalk+mech+htsvoice+speed+out_wav+out_log+all_pass+post_filter
	c=subprocess.Popen(cmd, stdin=subprocess.PIPE)
	c.stdin.write(t)
	c.stdin.close()
	c.wait()
	aplay=['aplay', '-q', 'open_jtalk.wav']
	wr=subprocess.Popen(aplay)


def say_text():
	print '<音声合成したい文章を入力してください>'
	text = raw_input()
	open_jtalk(text)

if __name__ == '__main__':
	while (1):
		say_text()
