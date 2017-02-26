# -*- coding: utf-8 -*-

# Copyright © 2017, CNRS-LAAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


from __future__ import absolute_import, print_function, division

import threading


class SynchronousTask(threading.Thread):
    """SynchronousTask executes a function each time `next` is called.

    It can be used to run some task iteratively sychronized with an external
    signal.
    """
    def __init__(self, name, target):
        """Init SynchronousTask.

        Arguments:
            name: task name. it can be None.
            action: callback fn(*action_args, **action_kwargs) called each time
                next(action_args, action_kwargs) is called.
        """
        super(SynchronousTask, self).__init__(name=name)
        self._cancel_event = threading.Event()
        self._next_event = threading.Event()

        self._action = target
        self._action_args = ()
        self._action_kwargs = {}

        self._period = 0.001  # waiting period

    def run(self):
        """Start SynchronousTask."""
        while True:
            while not self._next_event.wait(timeout=self._period):
                if self._cancel_event.is_set():
                    return
            self._action(*self._action_args, **self._action_kwargs)
            self._action_args = ()
            self._action_kwargs = {}
            self._next_event.clear()

    def cancel(self):
        """Cancel the task."""
        self._cancel_event.set()

    def next(self, action_args=(), action_kwargs={}):
        """Perform next iteration of task.

        If task is not started, it does nothing.

        Arguments:
            action_args: action args.
            action_kwargs: action kwargs.

        Raises:
            RuntimeError if previou iteration haven't finished.
        """
        if not self.is_alive():
            return
        self._action_args = action_args
        self._action_kwargs = action_kwargs
        if self._next_event.is_set():
            raise RuntimeError("Previous iteration haven't finished yet")
        else:
            self._next_event.set()

    def is_ready(self):
        """Returns wheter it is ready to start an new iteration."""
        return not self._next_event.is_set()


def main():
    """Test function for SynchronousTask."""
    def operation(it):
        print(str(it))
        time.sleep(1)
        print("waiting...")

    task = SynchronousTask("name", operation)
    task.daemon = True

    iteration = 0

    print("write exit to exit")

    task.start()
    while True:
        entrada = raw_input()
        iteration += 1
        if entrada == "exit":
            #task.cancel()
            break
        try:
            task.next((iteration,))
        except RuntimeError:
            iteration -= 1
            print("wait for the task to finish!")

if __name__ == "__main__":
    main()
