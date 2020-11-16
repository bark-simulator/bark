# Copyright (c) 2020 fortiss GmbH
#
# Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
# Tobias Kessler
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import tempfile
import os
import shutil
import uuid
import subprocess
import logging

from bark.core.geometry import *
from bark.core.viewer import *
from bark.core.models.dynamic import *
from bark.runtime.viewer.viewer import BaseViewer


class VideoRenderer(BaseViewer):
    def __init__(self, renderer, world_step_time, render_intermediate_steps=None , params=None, video_name=None, fig_path=None, clear_figures=True, **kwargs):
        super(VideoRenderer, self).__init__(params=params)
        self.renderer = renderer
        self.world_step_time = world_step_time
        self.render_intermediate_steps = render_intermediate_steps

        video_dir_name = "vid_render_frames_{}".format(uuid.uuid4()) if video_name is None else video_name
        self.fig_path = fig_path if fig_path is not None else tempfile.gettempdir()
        self.video_frame_dir = os.path.join(self.fig_path, video_dir_name)
        if clear_figures and os.path.exists(self.video_frame_dir):
            print("video path already exists, clear existing one")
            self.reset()
        os.makedirs(self.video_frame_dir, exist_ok=True)

        self.frame_count = 0

    def drawWorld(self, world, eval_agent_ids=None, scenario_idx=None, debug_text=False):
        if self.render_intermediate_steps is None:
            self._renderWorld(world, eval_agent_ids, scenario_idx)
        else:
            world_time = world.time
            executed_world = world
            for _ in range(0,self.render_intermediate_steps):
                executed_world = executed_world.GetWorldAtTime(world_time)
                self._renderWorld(executed_world, eval_agent_ids, scenario_idx)
                world_time = world_time + self.world_step_time/self.render_intermediate_steps

    def drawText(self, **kwargs):
        self.renderer.drawText(**kwargs)

    def _renderWorld(self, world, eval_agent_ids=None, scenario_idx=None, debug_text=False):
        image_path = os.path.join(self.video_frame_dir, "{:03d}.png".format(self.frame_count))
        self.renderer.drawWorld(world=world, eval_agent_ids=eval_agent_ids,
                                filename=image_path, scenario_idx=scenario_idx, debug_text=debug_text)
        self.frame_count = self.frame_count + 1

    def reset(self):
        shutil.rmtree(os.path.abspath(self.video_frame_dir))

    def export_video(self, filename, remove_image_dir=True):
        if self.render_intermediate_steps is None:
            framerate = 1/self.world_step_time
        else:
            framerate =  1/(self.world_step_time/self.render_intermediate_steps)

        if os.path.isfile(filename):
            os.remove(filename)
        if not os.path.exists(os.path.dirname(filename)):
            os.makedirs(os.path.dirname(filename))
        cmd = "ffmpeg -nostdin -y -framerate {} -i \'{}/%03d.png\' -vcodec h264 -force_key_frames 'expr:gte(t,n_forced*{})' -acodec pcm_s16le -s 1920x1080 -r 30 -b:v 36M -pix_fmt yuv420p -f mp4 \'{}.mp4\'".format(
            framerate, self.video_frame_dir, 1 / framerate, os.path.abspath(filename))
        retval = subprocess.call(cmd, shell=True)
        if retval:
          logging.error("Error during video export.")

        if remove_image_dir:
            shutil.rmtree(os.path.abspath(self.video_frame_dir))

    def clear(self):
      self.renderer.clear()



