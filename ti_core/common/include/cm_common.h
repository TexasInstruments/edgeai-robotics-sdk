/*
 *
 * Copyright (c) 2021 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to make,
 * have made, use, import, offer to sell and sell ("Utilize") this software subject to the
 * terms herein.  With respect to the foregoing patent license, such license is granted
 * solely to the extent that any such patent is necessary to Utilize the software alone.
 * The patent license shall not apply to any combinations which include this software,
 * other than combinations with devices manufactured by or for TI ("TI Devices").
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *
 * *       No reverse engineering, decompilation, or disassembly of this software is
 * permitted with respect to any software provided in binary form.
 *
 * *       any redistribution and use are licensed by TI for use only with TI Devices.
 *
 * *       Nothing shall obligate TI to provide you with source code for the software
 * licensed and provided to you in object code.
 *
 * *       any redistribution a
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any object code compiled from the source code
 * and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 *
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI'S LICENSORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI'S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#if !defined(_CM_COMMON_CNTXT_H_)
#define _CM_COMMON_CNTXT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <cassert>

#include <TI/tivx.h>
#include <TI/j7_tidl.h>
#include <TI/hwa_kernels.h>
#include <TI/dl_kernels.h>
#include <TI/hwa_vpac_ldc.h>
#include <TI/hwa_vpac_msc.h>
#include <TI/hwa_vpac_viss.h>
#include <TI/hwa_vpac_nf.h>
#include <TI/hwa_dmpac_dof.h>
#include <TI/hwa_dmpac_sde.h>

#if !defined(SOC_AM62A) && !defined(SOC_AM62)
#include <perception/perception.h>
#endif // !defined(SOC_AM62A) || !defined(SOC_AM62)

#include <edgeai_dl_inferer/ti_dl_inferer.h>
#include <edgeai_dl_inferer/ti_pre_process_config.h>
#include <edgeai_dl_inferer/ti_post_process_config.h>
#include <utils/include/ti_logger.h>

using namespace std;
using namespace ti::utils;
using namespace ti::dl_inferer;
using namespace ti::pre_process;
using namespace ti::post_process;

#define clip3(x, min, max) ((x) > (max)?(max):((x) < (min)?(min):(x)))
#define CM_normalize(x, mean, scale) (((x) - (mean))*(scale))

/**
 * \defgroup group_ticore_common TI Robotics SDK common
 *
 * \brief Provides common functions for image loading/saving, data copy,
 *        DL buffer creation, etc.
 *
 * \ingroup  group_ticore_base
 *
 */

namespace ti_core_common
{

/**
 * \brief Constant for max pipeline depth
 * \ingroup group_ticore_common
 */
#define GRAPH_MAX_PIPELINE_DEPTH (8)

/**
 * \brief Constant for max file name length
 * \ingroup group_ticore_common
 */
#define CM_MAX_FILE_LEN          (1024U)

/**
 * \brief Constant for use in handling strings. The specified size is in bytes.
 * \ingroup group_ticore_common
 */
#define CM_MAX_LINE_LEN          (256U)

/**
 * \brief Constant for use in validating the pipeline depth.
 * \ingroup group_ticore_common
 */
#define CM_MAX_PIPELINE_DEPTH    (8U)


/**
 * \brief Constant for max tensor dimension
 * \ingroup group_ticore_common
 */
#define CM_MAX_TENSOR_DIMS       (4u)


/**
 * \brief Input image format
 * \ingroup group_ticore_common
 */
typedef enum {
    CM_IMG_FORMAT_Y    = 0,
    CM_IMG_FORMAT_NV12 = 1,
    CM_IMG_FORMAT_UYVY = 2
} CM_IMG_FORMAT;


/**
 * \brief Returns Core name where a node will be deployed on the target given the core name from a config
 *
 * \param [in] appCoreName core name where a node will be deployed from a config
 *
 * \return Core name on the target
 *
 * \ingroup group_ticore_common
 */
const char *CM_getCoreName(const char *appCoreName);


/**
 * \brief Add node parameter to a set of graph parameters
 *
 * \param [in] graph graph object
 *
 * \param [in] node whose parameter is added to graph parameters
 *
 * \param [in] nodeParamIndex index of node parameters that is added to graph parameters
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_common
 */
vx_status CM_addParamByNodeIndex(vx_graph  graph,
                                 vx_node   node,
                                 vx_uint32 nodeParamIndex);


/**
 * \brief Returns the payload pointer from vx_user_data_object.
 *
 * \param [in] obj vx_user_data_object
 *
 * \return Pointer to the payload
 *
 * \ingroup group_ticore_common
 */
void * CM_getUserObjDataPayload(vx_user_data_object   obj);

/**
 * \brief Function for loading an image from a file.
 *
 * \param [in] fname Name of the file to read the image.
 *
 * \param [out] image OpenVX image object that image data is loaded.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_common
 */
vx_status CM_loadImage(char *fname, vx_image image);

/**
 * \brief Function for saving an image.
 *
 * \param [in] fname Name of the file for saving the image.
 *
 * \param [out] image OpenVX image object with image data to save.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_common
 */
vx_status CM_saveImage(const char *fname, vx_image image);

/**
 * \brief Function for copying OpenVX imge object to OpenVX image object.
 *
 * \param [in] srcImage source image object
 *
 * \param [in] dstImage output image object
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_common
 */
vx_status CM_copyImage2Image(vx_image srcImage, vx_image dstImage);

/**
 * \brief Function for copying data from a data pointer to OpenVX image object
 *
 * \param [in] data_ptr_src source image data
 *
 * \param [in] dstImage     image object that image data is copied to
 *
 * \param [in] hSrcOfst     horizontal offset from which the image data is copied
 *
 * \param [in] vSrcOfst     vertical offset from which the image data is copied
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_common
 *
 */
vx_status CM_copyData2Image(const uint8_t * data_ptr_src,
                            vx_image        dstImage,
                            int16_t         hSrcOfst,
                            int16_t         vSrcOfst);


/**
 * \brief Function to convert a YUV420 image to planar RGB format.
 *
 * \param [out] rgbImage RGB image data.
 *
 * \param [in] yuvImage YUV raw image data. yuvImage[0] will point to the Y
 *                      plane date and yuvImage[1] will point to the UV plane
 *                      data.
 *
 * \param [in] width Width of the image in pixels.
 *
 * \param [in] height Height of the image in pixels.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_common
 */
vx_status CM_convertYUV2RGB(uint8_t        *rgbImage,
                            const uint8_t  *yuvImage[2],
                            int32_t         width,
                            int32_t         height);


/**
 * \brief Function to extract image data from an openVX YUV image object. The
 *        data can be extracted and converted to RGB format using 'rgbFlag'
 *        parameter.
 *
 * \param [out] outImageData Output image data. The size of this buffer needs
 *                           to be properly dimensioned, as follows:
 *                           - size = 1.5 * width * height for YUV data, i.e. if
 *                             rgbFlag == 0
 *                           - size = 3 * width * height for RGB data, i.e. if
 *                             rgbFlag == 1
 *
 * \param [in] yuvImage OpenVX Image object carrying YUV image.
 *
 * \param [in] width Width of the image in pixels.
 *
 * \param [in] height Height of the image in pixels.
 *
 * \param [in] numPlanes Number of planes in the input Image object (yuvImage)
 *
 * \param [in] rgbFlag Flag to indicate if the image data needs to be converted to
 *                     RGB format.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_common
 */
vx_status CM_extractImageData(uint8_t         *outImageData,
                              const vx_image   yuvImage,
                              uint32_t         width,
                              uint32_t         height,
                              uint8_t          numPlanes,
                              uint8_t          rgbFlag);

/**
 * \brief Function to extract 8-bit raw tensor data from an openVX tensor object.
 *        Other data type is not supported yet.
 *
 * \param [out] outTensorData Output 8-bit tensor data. The size of this buffer is
 *                            "size".
 *
 * \param [in]  tensor OpenVX Tensor object
 *
 * \param [in]  size  size of raw tensor data in bytes.
 *
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_common
 */
vx_status CM_extractTensorData(uint8_t         *outTensorData,
                               const vx_tensor  tensor,
                               uint32_t         size);


/**
 * \brief Function to read a input binary file and fill 1-D tensor object's data
 *
 * \param [in,out] in_tensor tensor that reads in a file
 *
 * \param [in]  in_file input file name
 *
 * \return The number of bytes
 *
 * \ingroup group_ticore_common
 */
vx_int32  CM_fill1DTensor(vx_tensor        in_tensor,
                          const vx_char  * in_file);


/**
 * \brief Function to read two binary files and fill 1-D tensor object's data
 *
 * \param [in,out] in_tensor tensor that reads in file files
 *
 * \param [in]  in_file1 input file name 1
 *
 * \param [in]  in_file2 input file name 2
 *
 * \return The number of bytes
 *
 * \ingroup group_ticore_common
 */
vx_int32  CM_fill1DTensorFrom2Bin(vx_tensor        in_tensor,
                                  const vx_char  * in_file1,
                                  const vx_char  * in_file2);


#if !defined(SOC_AM62A) && !defined(SOC_AM62)
/**
 * \brief Function to extract point cloud (XYZRGB) data from an OpenVX user data object.
 *        When copyPointFlag = 1, copy points to outPcData. Otherwise, only get numPoints.
 *
 * \param [out]     outPcData Output point cloud data, which inclues X,Y,Z and R, G, B for each point.
 *                            The size of each point is 15 bytes.
 *
 * \param [in]      pointCloud OpenVX user object for PTK_PointCloud
 *
 * \param [in]      pointSize  size of one point in byte
 *
 * \param [in,out]  numPoints  the number of points in pointCloud object.
 *
 * \param [in]      copyPointFlag whether to copy points to outPcData
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_common
 */

vx_status CM_extractPointCloudData(uint8_t                  *outPcData,
                                   const vx_user_data_object pointCloud,
                                   uint32_t                  pointSize,
                                   uint32_t                 *numPoints,
                                   uint8_t                   copyPointFlag);
#endif // !defined(SOC_AM62A) || !defined(SOC_AM62)

/**
 * \brief Function to create input and output buffers for DL inferer
 *
 * \param [out]     inferer DL inferer object point
 *
 * \param [in]      ifInfoList DL tensor info
 *
 * \param [in,out]   vecVar  DL tensor pointer where buffers are allocated
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_common
 */
vx_status CM_createDlBuffers(DLInferer            *inferer,
                             const VecDlTensor    *ifInfoList,
                             VecDlTensorPtr       &vecVar);


/**
 * \brief Function to copy tensor data
 *
 * \param [in] inData input buffer or tensor
 *
 * \param [out] outData output buffer or tensor
 *
 * \param [in] numElem the number of elements to copy
 *
 * \return
 *
 * \ingroup group_ticore_common
 */

template <typename inT, typename outT>
void CM_copyTensorData(inT  *inData, outT *outData, int32_t  numElem)
{
    for (uint32_t i = 0; i < numElem; i++)
    {
        *outData++ = static_cast<outT>(*inData++);
    }
}


} // namespace ti_core_common

#endif /* _CM_COMMON_CNTXT_H_ */

