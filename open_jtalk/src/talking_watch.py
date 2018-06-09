#!/usr/bin/env python
#encoding: utf-8
import rospy, subprocess, os, datetime
from std_msgs.msg import String

def open_jtalk(t):
	open_jtalk=['open_jtalk']
	mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
	htsvoice=['-m', '/home/ohishikouta/catkin_ws/src/open_jtalk/voice/mei/mei_normal.htsvoice']
	speed=['-r', '1.0']
	out_wav=['-ow', 'talking_watch.wav']
	out_log=['-ot', 'talking_watch.log']
	cmd=open_jtalk+mech+htsvoice+out_wav+out_log+speed
	c=subprocess.Popen(cmd, stdin=subprocess.PIPE)
	c.stdin.write(t)
	c.stdin.close()
	c.wait()
	aplay=['aplay', 'talking_watch.wav']
	wr=subprocess.Popen(aplay)
	
def callback(data):
	pub = rospy.Publisher('/speech_word', String, queue_size=100)
	speech_word = ""
	if "今" in data.data:
		if "時間" or "時刻" or "何時" in data.data:
			date=datetime.datetime.today()
			speech_word="%s時%s分%s秒です。"%(date.hour, date.minute, date.second)
			print speech_word
			open_jtalk(speech_word)
			pub.publish(String(speech_word))
		
def julius_subscriber():
	rospy.init_node('julius_subscriber', anonymous=True)
	rospy.Subscriber("/recognition_word", String, callback)
	rospy.spin()
	
if __name__ == '__main__':
	julius_subscriber()
	#os.system("rm talking_watch.wav")
