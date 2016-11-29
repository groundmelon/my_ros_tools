#!/usr/bin/env python

import urwid
import urwid.raw_display
import urwid.web_display

import dialog
import os
import rosgraph
import re
from collections import OrderedDict
import argparse
import socket


def sysout(s):
    # with open('/dev/pts/32', 'a') as f:
    #     f.write(s)
    #     f.write(os.linesep)
    pass


def get_publishing_topics():
    d = dict()

    try:
        master = rosgraph.masterapi.Master('/rostopic')
        topics = [x[0] for x in master.getPublishedTopics('/')]
        topics.sort()
    except socket.error:
        raise Exception('Connect to roscore failed. Is the roscore online?')

    prog = re.compile("^(/.+?)/|^(/.+?)$")

    for t in topics:
        m = prog.match(t)
        if m:
            node = None
            if m.group(1):
                node = m.group(1)
            if m.group(2):
                node = m.group(2)

            if node:
                if node in d:
                    d[node].append(t)
                else:
                    d[node] = [t]
    return d


class TopicCheckBox(urwid.CheckBox):
    # _metaclass_ = urwid.signals.MetaSignals

    def __init__(self, topic, node):
        self.topic = topic
        super(TopicCheckBox, self).__init__(
            topic, state=False, has_mixed=False, on_state_change=None)

        urwid.connect_signal(node, 'change', self.on_node_changed)

    def on_node_changed(self, widget, value):
        sysout("Father node changed for [{}] with value [{}]".format(self.topic, type(value)))
        if isinstance(value, bool):
            self.set_state(value, do_callback=False)
        else:
            pass


class NodeCheckBox(urwid.CheckBox):
    # _metaclass_ = urwid.signals.MetaSignals

    def __init__(self, node_name, topics):
        self.node_name = node_name
        self.child_widgets = [
            urwid.Padding(TopicCheckBox(topic, self), align='left', left=2) for topic in topics]
        super(NodeCheckBox, self).__init__(
            node_name, state=False, has_mixed=True, on_state_change=None)

        for chkbox in self.get_child_chkboxes():
            urwid.connect_signal(chkbox, 'change', self.child_changed)

    def child_changed(self, widget, value):
        sysout("child changed for {}, {}, {}".format(self.node_name, value, widget.state))
        self.refresh_by_children(widget, value)

    def refresh_by_children(self, widget=None, value=None):
        true_cnt = 0
        false_cnt = 0
        for chkbox in self.get_child_chkboxes():
            s = None
            if chkbox is widget:
                s = value
            else:
                s = chkbox.state

            if s is True:
                true_cnt = true_cnt + 1
            if s is False:
                false_cnt = false_cnt + 1

        desired_state = self.state
        if true_cnt == 0 and false_cnt != 0:
            desired_state = False
        elif true_cnt != 0 and false_cnt == 0:
            desired_state = True
        else:
            desired_state = 'mixed'

        # if desired_state != self.state:
        self.set_state(desired_state, do_callback=False)

    def get_child_chkboxes(self):
        for w in self.child_widgets:
            yield w.original_widget

    @classmethod
    def create(cls, node_name, topics):
        return urwid.Padding(NodeCheckBox(node_name, topics), align='left', left=0)


class Application:

    def __init__(self, output_prefix=None):
        self.exit_message = None

        self.output_prefix = output_prefix

        text_header = (u"Command Generation for 'rosbag record' ")
        buttons = OrderedDict()
        buttons['quit'] = urwid.Button(u"Quit(ESC/q)", self.on_quit)
        buttons['save'] = dialog.PopUpButton(u"Save(F2)",
                                             self.on_save,
                                             prompt_text=u"Input file name:",
                                             default_text=u"record.sh")
        buttons['mark'] = urwid.Button(u"Mark all(F3)", self.on_mark_all)
        buttons['unmark'] = urwid.Button(u"Unmark all(F4)", self.on_unmark_all)
        buttons['refresh'] = urwid.Button(u"Refresh(F5)", self.on_refresh)
        self.buttons = buttons

        # blank = urwid.Divider()

        header = urwid.AttrWrap(urwid.Text(text_header, align='center'), 'header')

        button_bar = urwid.GridFlow(
            [urwid.AttrWrap(btn, 'buttn', 'buttnf')
             for (key, btn) in buttons.iteritems()], 18, 3, 1, 'left')
        status_bar = urwid.AttrWrap(urwid.Text(u"status bar"), 'footer')
        footer = urwid.Pile([button_bar, status_bar])

        self.listwalker = urwid.SimpleListWalker(self.create_listbox_from_ros())
        listbox = urwid.ListBox(self.listwalker)
        body = urwid.AttrWrap(listbox, 'body')

        self.frame = urwid.Frame(body=body, header=header, footer=footer)

        self.frame_focus_table = dict()
        self.frame_focus_table[body] = 'footer'
        self.frame_focus_table[footer] = 'body'

        palette = [
            ('body', 'white', 'black', 'standout'),
            ('reverse', 'light gray', 'black'),
            ('header', 'white', 'dark blue', 'bold'),
            ('footer', 'black', 'light gray'),
            ('important', 'dark blue', 'light gray', ('standout', 'underline')),
            ('editfc', 'white', 'dark blue', 'bold'),
            ('editbx', 'light gray', 'dark blue'),
            ('editcp', 'black', 'light gray', 'standout'),
            ('bright', 'dark gray', 'light gray', ('bold', 'standout')),
            ('buttn', 'black', 'light cyan'),
            ('buttnf', 'white', 'dark blue', 'bold'),
            ('popbg', 'white', 'dark gray'),
        ]

        self.show_msg = status_bar.set_text

        # use appropriate Screen class
        if urwid.web_display.is_web_request():
            screen = urwid.web_display.Screen()
        else:
            screen = urwid.raw_display.Screen()

        self.mainloop = urwid.MainLoop(self.frame, palette, screen,
                                       unhandled_input=self.unhandled, pop_ups=True)

    # UI functions
    def on_quit(self, button):
        self.show_msg(button.get_label())

    def on_save(self, button):
        self.start_save()

    def on_mark_all(self, button):
        self.do_set_all(True)

    def on_unmark_all(self, button):
        self.do_set_all(False)

    def on_refresh(self, button):
        self.do_refresh()

    def start_save(self):
        self.buttons['save'].do_inputbox(self.do_save)

    def button_press(self, button):
        self.show_msg(button.get_label())

    def unhandled(self, key):
        if key == 'esc' or key == 'q':
            raise urwid.ExitMainLoop()
        elif key == 'tab':
            self.frame.focus_position = self.frame_focus_table[self.frame.focus]
        elif key == 'f2':
            self.start_save()
        elif key == 'f3':
            self.do_set_all(True)
        elif key == 'f4':
            self.do_set_all(False)
        elif key == 'f5':
            self.do_refresh()
        else:
            self.show_msg("Pressed: {}".format(key))

    # functional functions
    def get_listbox_chkboxes(self):
        for w in self.listwalker:
            yield w.original_widget

    def do_refresh(self):
        self.listwalker[:] = self.create_listbox_from_ros()
        self.show_msg("Refreshed! new one has {} elements".format(len(self.listwalker)))

    def do_set_all(self, value):
        for chkbox in self.get_listbox_chkboxes():
            chkbox.set_state(value, do_callback=False)

    def do_save(self, fname):

        s = " ".join(["rosbag record",
                      " ".join(self.get_selections())])

        if self.output_prefix:
            s += " -o {}".format(self.output_prefix)

        s += os.linesep

        self.exit_message = s
        if not fname:
            fname = "record.sh"
        with open(fname, "w") as f:
            f.write(s)
        self.show_msg("{} generated".format(fname))

    def create_listbox_from_ros(self):
        listbox_content = []
        for node, topics in get_publishing_topics().iteritems():
            node_chkbox = NodeCheckBox.create(node, topics)
            listbox_content.append(node_chkbox)
            for x in node_chkbox.original_widget.child_widgets:
                listbox_content.append(x)
        return listbox_content

    def get_selections(self):
        lst = []
        for chkbox in self.get_listbox_chkboxes():
            if isinstance(chkbox, TopicCheckBox) and chkbox.state is True:
                lst.append(chkbox.topic)
        return lst


if '__main__' == __name__:
    parser = argparse.ArgumentParser(
        description=u"A program that helps generating \"rosbag record TOPIC1 TOPIC2 ... TOPICN\" command")
    parser.add_argument("-o", "--prefix", help=u"argument for \"rosbag record -o\"")
    args = parser.parse_args()

    app = Application(args.prefix)
    app.mainloop.run()
    if app.exit_message:
        print(app.exit_message)
