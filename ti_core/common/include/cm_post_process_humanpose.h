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

#ifndef _POST_PROCESS_IMAGE_HUMAN_POSE_ESTIMATION_H_
#define _POST_PROCESS_IMAGE_HUMAN_POSE_ESTIMATION_H_

/* Module headers. */
#include <cm_post_process_image.h>

/**
 * \defgroup group_edgeai_cpp_apps_human_pose_estimation Human Pose Estimation post-processing
 *
 * \brief Class implementing the image based Human Pose estimation post-processing
 *        logic.
 *
 * \ingroup group_edgeai_cpp_apps_post_proc
 */

namespace ti_core_common
{
    /** Post-processing for image based human pose estimation.
     *
     * \ingroup group_edgeai_cpp_apps_human_pose_estimation
     */
    class CmPostprocessHumanPoseEstimation : public CmPostprocessImage
    {
        public:
            /** Constructor.
             *
             * @param config Configuration information not present in YAML
             * @param debugConfig Debug Configuration for passing to post process class
             */
            CmPostprocessHumanPoseEstimation(const PostprocessImageConfig  &config);

            /** Function operator
             *
             * This is the heart of the class. The application uses this
             * interface to execute the functionality provided by this class.
             *
             * @param inData Input data frame on which results are overlaid
             * @param outData Output data frame with results overlaid
             * @param results Detection output results from the inference
             */
            void *operator()(void              *inData,
                             void              *outData,
                             VecDlTensorPtr    &results);

            /** Destructor. */
            ~CmPostprocessHumanPoseEstimation();

        private:
            /** Multiplicative factor to be applied to X co-ordinates. */
            float                                           m_scaleX{1.0f};

            /** Multiplicative factor to be applied to Y co-ordinates. */
            float                                           m_scaleY{1.0f};

            float                                           m_vizThreshold{0.5f};

        private:
            /**
             * Assignment operator.
             *
             * Assignment is not required and allowed and hence prevent
             * the compiler from generating a default assignment operator.
             */
            CmPostprocessHumanPoseEstimation &
                operator=(const CmPostprocessHumanPoseEstimation& rhs) = delete;
    };

} // namespace ti::edgeai::common

#endif /* _POST_PROCESS_IMAGE_HUMAN_POSE_ESTIMATION_H_ */