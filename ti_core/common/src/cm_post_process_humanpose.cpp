/*
 *  Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Module headers. */
#include <cm_post_process_humanpose.h>
#include <utils/include/ti_logger.h>
using namespace ti::utils;

namespace ti_core_common
{

typedef struct Keypoint
{
    float kp_x;
    float kp_y;
    float kp_conf;
}Keypoint;

CmPostprocessHumanPoseEstimation::CmPostprocessHumanPoseEstimation(const PostprocessImageConfig  &config):
    CmPostprocessImage(config)
{
    m_vizThreshold = config.vizThreshold;
    LOG_INFO("Created\n");
}

void *CmPostprocessHumanPoseEstimation::operator()(void           *inData,
                                                   void           *outData,
                                                   VecDlTensorPtr &results)
{
    auto                    *resultRo = results[0];
    float                   *data = (float*)resultRo->data;
    float                   *out  = reinterpret_cast<float *>(outData);

    float                   *count = out+2;

    int                     tensorHeight = resultRo->shape[resultRo->dim - 2];
    int                     tensorWidth  = resultRo->shape[resultRo->dim - 1];
    int32_t                 num_boxes    = 0;

    if (!m_config.normDetect)
    {
        *out++ = static_cast<float>(m_config.inDataWidth);
        *out++ = static_cast<float>(m_config.inDataHeight);
    }
    else
    {
        *out++ = 1.0;
        *out++ = 1.0;
    }
    out++;

    for (int i = 0; i < tensorHeight; i++)
    {
        float   score;
        int32_t label;
        int32_t box[4];

        score = data[i * tensorWidth + 4];
        label = data[i * tensorHeight + 5];

        if (score >  m_vizThreshold)
        {

            box[0] = data[i * tensorWidth + 0] ;
            box[1] = data[i * tensorWidth + 1] ;
            box[2] = data[i * tensorWidth + 2] ;
            box[3] = data[i * tensorWidth + 3] ;

            *out++ = static_cast<float>(box[0]);
            *out++ = static_cast<float>(box[1]);
            *out++ = static_cast<float>(box[2]);
            *out++ = static_cast<float>(box[3]);
            *out++ = score;
            *out++ = static_cast<float>(label);

            for(int j = 6; j < tensorWidth; j++)
            {
                *out++ = data[i * tensorWidth + j];
            }

            num_boxes++;
        }

    }

    *count = static_cast<float>(num_boxes);

    return outData;

}

CmPostprocessHumanPoseEstimation::~CmPostprocessHumanPoseEstimation()
{
}

}