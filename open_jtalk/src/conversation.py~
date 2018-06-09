#!/usr/bin/env python
#encoding: utf-8
import rospy, subprocess, os, datetime
from std_msgs.msg import String

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
	speed=['-r', '1.0']
	out_wav=['-ow', 'julius_open_jtalk.wav']
	out_log=['-ot', 'julius_open_jtalk.log']
	cmd=open_jtalk+mech+htsvoice+out_wav+out_log+speed
	c=subprocess.Popen(cmd, stdin=subprocess.PIPE)
	c.stdin.write(t)
	c.stdin.close()
	c.wait()
	aplay=['aplay', 'julius_open_jtalk.wav']
	wr=subprocess.Popen(aplay)

	
def callback(data):
	if "おはよう" in data.data:
		open_jtalk("グッドモーニング。")
	
	if "こんにちは" in data.data:
		open_jtalk("ハロー。")
	
	if "ありがと" in data.data:
		open_jtalk("どういたしまして。")
			
	if "今" in data.data:
		if "時間" or "時刻" or "何時" in data.data:
			date=datetime.datetime.today()
			open_jtalk("%s時%s分%s秒です。"%(date.hour, date.minute, date.second))
		
	if "名前" in data.data:
		if "あなた" in data.data:
			open_jtalk("私の名前はソビットです。")	
	
def julius_subscriber():
	rospy.init_node('julius_subscriber', anonymous=True)
	rospy.Subscriber("/recognition_word", String, callback)
	rospy.spin()
	

if __name__ == '__main__':
	julius_subscriber()
	os.system("rm julius_open_jtalk.wav")

