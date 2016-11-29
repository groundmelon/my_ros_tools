#!/usr/bin/env python

import rospy
import rostopic
import argparse


def get_avg_max_min(t):
    if not t:
        return {'max': float('nan'), 'min': float('nan'), 'avg': float('nan')}

    max_ = t[0]
    min_ = t[0]
    avg_ = 0.0

    for x in t:
        max_ = max(max_, x)
        min_ = min(min_, x)
        avg_ = avg_ + x

    return {'max': max_, 'min': min_, 'avg': avg_ / float(len(t))}


class Record:

    def __init__(self, ref, dst, nt):
        self.ref = ref
        self.dst = dst
        self.nt = nt


class TopicStatistic:

    def __init__(self, buffer_length, dst_topic=None, ref_topic=None):
        assert(dst_topic)
        self.dst_topic = dst_topic
        self.ref_topic = ref_topic
        self.buffer_length = buffer_length

        rospy.loginfo("statistics for \"{}\", using \"{}\" as reference".format(
            self.dst_topic, "ros.Time.now()" if not self.ref_topic else self.ref_topic))

        self.subscribe_topic(dst_topic, self.dst_topic_callback)
        if self.ref_topic:
            self.subscribe_topic(ref_topic, self.ref_topic_callback)

        self.buffer = []
        self.ref_msg = None

    def dst_topic_callback(self, msg):
        try:
            msg.header.stamp
        except AttributeError:
            raise Exception(
                "\"{}\"<{}> does not have header.stamp".format(self.dst_topic, str(type(msg))))

        now_time = rospy.Time.now()
        ref_time = None
        if self.ref_topic and self.ref_msg:
            ref_time = self.ref_msg.header.stamp
        if not self.ref_topic:
            ref_time = now_time

        if not ref_time:
            return

        self.buffer.append(Record(ref_time, msg.header.stamp, now_time))

    def ref_topic_callback(self, msg):
        try:
            msg.header.stamp
        except AttributeError:
            raise Exception(
                "\"{}\"<{}> does not have header.stamp".format(self.ref_topic, str(type(msg))))
        self.ref_msg = msg

    def subscribe_topic(self, topic, callback):
        msg_class, _, _ = rostopic.get_topic_class(topic)
        try:
            rospy.Subscriber(topic, msg_class, callback)
        except ValueError:
            raise Exception("topic \"{}\" is not found".format(topic))

    def process(self):
        start_idx = 0
        while start_idx < len(self.buffer):
            if (self.buffer[start_idx].nt + self.buffer_length) < rospy.Time.now():
                start_idx += 1
            else:
                break

        self.buffer = self.buffer[start_idx:-1]

        if not self.buffer:
            print(
                "delay avg: ----- min: ----- max: ----- | hz: 0.0000 | interval min: ----- max: ----- ")
            return

        delay_info = get_avg_max_min([(x.ref - x.dst).to_sec() for x in self.buffer])
        interval_info = get_avg_max_min(
            [(x[1].dst - x[0].dst).to_sec() for x in zip(self.buffer[0:-2], self.buffer[1:-1])])
        hz_info = {'hz': 1.0 / interval_info['avg'],
                   'max': interval_info['max'], 'min': interval_info['min']}

        d = {'delay': delay_info, 'hz': hz_info}

        print("delay avg:{: .3f} min:{: .3f} max:{: .3f} | hz: {:6.2f} | interval min:{: .3f} max:{: .3f} | n:{:3d}".format(
            d['delay']['avg'], d['delay']['min'], d['delay']['max'],
            d['hz']['hz'], d['hz']['min'], d['hz']['max'], len(self.buffer))
        )

        return d


def main(args):
    rospy.init_node('enhanced_rostopic_statistic')
    if not args.window:
        args.window = 10

    ts = TopicStatistic(rospy.Duration(args.window), dst_topic=args.TOPIC, ref_topic=args.ref)

    rospy.Timer(rospy.Duration(1.0), lambda x: ts.process())

    rospy.spin()


if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(
            description=u"Enhanced rostopic statistics.\n"
                        u"Example: rosrun my_ros_tools enhanced-rostopic-statistic.py /visual_odometry/odom --ref /autopilot/imu")
        parser.add_argument("TOPIC", help=u"topic name for statistic")
        parser.add_argument(
            "-r", "--ref", help=u"topic name for reference. If not pass this argument, ros.Time.now will be used.")
        parser.add_argument("-w", "--window", type=float, help=u"duration of window for statistic")
        args = parser.parse_args()
        main(args)
    except rospy.ROSInterruptException:
        pass
