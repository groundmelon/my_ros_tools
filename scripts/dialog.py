#!/usr/bin/python

import urwid
import os


class CustomEdit(urwid.Edit):
    _metaclass_ = urwid.signals.MetaSignals
    signals = ['done']

    def on_finish(self, widget):
        urwid.emit_signal(self, 'done', self, self.get_edit_text())

    def keypress(self, size, key):
        if key == 'enter':
            # if you dont need a reference to the CustomEdit instance you can drop the 3rd argument
            self.on_finish(self)
            return
        elif key == 'esc':
            super(CustomEdit, self).set_edit_text('')
            self.on_finish(self)
            return
        return urwid.Edit.keypress(self, size, key)


class PopUpInputDialog(urwid.WidgetWrap):

    """A dialog that appears with nothing but a close button """
    signals = ['close']

    def __init__(self, prompt_text=u"Input:", default_text=u""):
        close_button = urwid.Button("OK")
        prompt = urwid.Text(prompt_text)
        edit_field = CustomEdit(caption=u'',
                                edit_text=default_text,
                                multiline=False,
                                align='left',
                                wrap='space',
                                allow_tab=False,
                                edit_pos=None,
                                layout=None,
                                mask=None)

        prompt_wrap = urwid.AttrMap(prompt, 'header')
        close_button_wrap = urwid.AttrMap(close_button, 'buttn', 'buttnf')
        edit_field_wrap = urwid.AttrMap(edit_field, 'editcp')

        urwid.connect_signal(close_button, 'click', edit_field.on_finish)
        urwid.connect_signal(edit_field, 'done', self.on_close)

        pile = urwid.Pile([prompt_wrap,
                           edit_field_wrap,
                           urwid.Padding(close_button_wrap, 'center', 6)])
        fill = urwid.Filler(urwid.Padding(pile, 'center', left=1, right=1))
        self.__super.__init__(urwid.AttrWrap(fill, 'popbg'))

    def on_close(self, widget, s):
        urwid.emit_signal(self, "close", self, s)


class PopUpButton(urwid.PopUpLauncher):

    def __init__(self, *args, **kwargs):
        self.__super.__init__(urwid.Button(*args))
        self.callback = None
        self.popup_kwargs = kwargs

    def do_inputbox(self, callback, **kwargs):
        self.open_pop_up()
        self.callback = callback

    def get_result(self):
        return self.result

    def create_pop_up(self):
        pop_up = PopUpInputDialog(**self.popup_kwargs)
        urwid.connect_signal(pop_up, 'close', self.show_result_and_close)
        return pop_up

    def get_pop_up_parameters(self):
        return {'left': 0, 'top': -5, 'overlay_width': 20, 'overlay_height': 5}

    def show_result_and_close(self, button, s):
        self.close_pop_up()
        if self.callback:
            self.callback(s)


if __name__ == "__main__":
    palette = [
        ('body', 'white', 'light gray', 'standout'),
        ('reverse', 'light gray', 'black'),
        ('header', 'white', 'dark red', 'bold'),
        ('footer', 'white', 'dark blue'),
        ('important', 'dark blue', 'light gray', ('standout', 'underline')),
        ('editfc', 'white', 'dark blue', 'bold'),
        ('editbx', 'light gray', 'dark blue'),
        ('editcp', 'black', 'light gray', 'standout'),
        ('bright', 'dark gray', 'light gray', ('bold', 'standout')),
        ('buttn', 'black', 'dark cyan'),
        ('buttnf', 'white', 'dark blue', 'bold'),
        ('popbg', 'white', 'dark gray'),
    ]

    btn = PopUpButton("click-me")
    fill = urwid.Filler(urwid.Padding(btn, 'center', 15))
    loop = urwid.MainLoop(
        fill,
        palette,
        pop_ups=True)
    loop.run()
