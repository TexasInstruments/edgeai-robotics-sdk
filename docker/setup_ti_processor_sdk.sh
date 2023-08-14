#!/bin/bash

#  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# build & install perf_stats (one-time)
perf_stats_path=/opt/edgeai-gst-apps/scripts/perf_stats/bin/Release/perf_stats

# disable Neo-DLR phone-home feature
# echo '{"enable_phone_home": false}' > /usr/local/lib/python3.8/dist-packages/dlr/counter/ccm_config.json

if [ -f "$perf_stats_path" ]; then
    ln -snf /opt/edgeai-gst-apps/scripts/perf_stats/bin/Release/perf_stats /usr/local/bin/perf_stats
else
    CWD=`pwd`
    cd /opt/edgeai-gst-apps/scripts/perf_stats
    rm -rf build && mkdir build && cd build && cmake .. && make && cd $CWD
    ln -snf /opt/edgeai-gst-apps/scripts/perf_stats/bin/Release/perf_stats /usr/local/bin/perf_stats
fi

if [ "$UBUNTU_VER" == "20.04" ]; then
    # settings for edgeai-gst-plugins: for export LD_PRELOAD to GLdispatch and set LD_LIBRARY_PATH
    echo export LD_PRELOAD=$LD_PRELOAD:/usr/lib/aarch64-linux-gnu/libGLdispatch.so.0 >> ~/.bashrc
    echo export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib:/usr/lib/aarch64-linux-gnu:/usr/lib/edgeai-tiovx-modules:/usr/lib/aarch64-linux-gnu/gstreamer-1.0 >> ~/.bashrc
fi
