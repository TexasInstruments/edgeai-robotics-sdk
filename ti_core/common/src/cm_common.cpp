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
#include <cm_common.h>
#include <utils/include/ti_logger.h>

using namespace ti::utils;
namespace ti_core_common
{

const char *CM_getCoreName(const char *appCoreName)
{
    if (!strcmp(appCoreName, "TIVX_TARGET_DSP1"))
    {
        return TIVX_TARGET_DSP1;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_DSP2"))
    {
        return TIVX_TARGET_DSP2;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_DSP_C7_1"))
    {
        return TIVX_TARGET_DSP_C7_1;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_MPU_0"))
    {
        return TIVX_TARGET_MPU_0;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_MPU_1"))
    {
        return TIVX_TARGET_MPU_1;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_MPU_2"))
    {
        return TIVX_TARGET_MPU_2;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_MPU_3"))
    {
        return TIVX_TARGET_MPU_3;
    }
    else if (!strcmp(appCoreName, "TIVX_TARGET_IPU1_0"))
    {
        return TIVX_TARGET_IPU1_0;
    }
#if !defined (SOC_AM62A) && !defined(SOC_J722S)
    else if (!strcmp(appCoreName, "TIVX_TARGET_IPU1_1"))
    {
        return TIVX_TARGET_IPU1_1;
    }
#endif //!defined (SOC_AM62A) && !defined(SOC_J722S)

    return NULL;
}

vx_status CM_addParamByNodeIndex(vx_graph  graph,
                                 vx_node   node,
                                 vx_uint32 nodeParamIndex)
{
    vx_parameter    param;
    vx_status       vxStatus;

    vxStatus = VX_SUCCESS;
    param = vxGetParameterByIndex(node, nodeParamIndex);

    if (param == NULL)
    {
        LOG_ERROR("vxGetParameterByIndex() failed\n");

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxAddParameterToGraph(graph, param);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("vxAddParameterToGraph() failed\n");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxReleaseParameter(&param);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("vxReleaseParameter() failed\n");
        }
    }

    return vxStatus;
}

void * CM_getUserObjDataPayload(vx_user_data_object   obj)
{
    void       *data;
    vx_map_id   mapId;
    vx_status   vxStatus;

    /* Map the user object. */
    vxStatus = vxMapUserDataObject(obj,
                                   0, // offset
                                   0, // size = 0 ==> entire buffer
                                   &mapId,
                                   &data,
                                   VX_READ_AND_WRITE,
                                   VX_MEMORY_TYPE_HOST,
                                   0);

    if ((vxStatus != (vx_status)VX_SUCCESS) || (data == NULL))
    {
        LOG_ERROR("vxMapUserDataObject() failed.\n");

        return NULL;
    }

    /* Unmap the user object. */
    vxStatus = vxUnmapUserDataObject(obj, mapId);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("vxUnmapUserDataObject() failed.\n");

        return NULL;
    }

    return data;
}


vx_status CM_loadImage(char *fname, vx_image image)
{
    vx_status  vxStatus = VX_SUCCESS;

    vx_rectangle_t             rect;
    vx_imagepatch_addressing_t image_addr;
    vx_map_id                  map_id;
    void                     * data_ptr;
    vx_uint32                  img_width;
    vx_uint32                  img_height;
    vx_df_image                img_format;
    vx_int32                   j;
    vx_int32                   data_read;

    FILE *fp= fopen(fname, "rb");

    if (fp == NULL)
    {
        LOG_ERROR("Unable to open input file [%s]\n", fname);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxQueryImage(image, VX_IMAGE_WIDTH, &img_width, sizeof(vx_uint32));
        vxQueryImage(image, VX_IMAGE_HEIGHT, &img_height, sizeof(vx_uint32));
        vxQueryImage(image, VX_IMAGE_FORMAT, &img_format, sizeof(vx_df_image));

        rect.start_x = 0;
        rect.start_y = 0;
        rect.end_x = img_width;
        rect.end_y = img_height;

        // Copy Luma or Luma+Chroma
        vxStatus = vxMapImagePatch(image,
                                   &rect,
                                   0,
                                   &map_id,
                                   &image_addr,
                                   &data_ptr,
                                   VX_WRITE_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   VX_NOGAP_X);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("vxMapImagePatch() failed\n");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        for (j = 0; j < image_addr.dim_y; j++)
        {
            data_read = fread(data_ptr, 1, image_addr.dim_x*image_addr.stride_x, fp);
            data_ptr = (vx_uint8 *)data_ptr + image_addr.stride_y;
        }
        vxUnmapImagePatch(image, map_id);
    }


    // Copy Chroma for NV12
    if (img_format == VX_DF_IMAGE_NV12)
    {
        vxStatus = vxMapImagePatch(image,
                                   &rect,
                                   1,
                                   &map_id,
                                   &image_addr,
                                   &data_ptr,
                                   VX_WRITE_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   VX_NOGAP_X);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("vxMapImagePatch() failed\n");
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            for (j = 0; j < img_height/2; j++)
            {
                data_read = fread(data_ptr, 1, image_addr.dim_x, fp);
                data_ptr = (vx_uint8 *)data_ptr + image_addr.stride_y;
            }

            vxUnmapImagePatch(image, map_id);
        }
    }

    fclose(fp);
    return vxStatus;
}

vx_status CM_saveImage(const char *fname, vx_image image)
{
    FILE                       *fp;
    void                       *data_ptr;
    vx_rectangle_t              rect;
    vx_imagepatch_addressing_t  image_addr;
    vx_map_id                   map_id;
    vx_uint32                   img_width;
    vx_uint32                   img_height;
    vx_uint32                   num_bytes;
    vx_status                   vxStatus;

    vxStatus = VX_SUCCESS;

    fp = fopen(fname, "wb");

    if (fp == NULL)
    {
        LOG_ERROR("Unable to open input file [%s]\n", fname);

        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxQueryImage(image, VX_IMAGE_WIDTH, &img_width, sizeof(vx_uint32));
        vxQueryImage(image, VX_IMAGE_HEIGHT, &img_height, sizeof(vx_uint32));

        rect.start_x = 0;
        rect.start_y = 0;
        rect.end_x = img_width;
        rect.end_y = img_height;
        vxStatus = vxMapImagePatch(image,
                                   &rect,
                                   0,
                                   &map_id,
                                   &image_addr,
                                   &data_ptr,
                                   VX_WRITE_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   VX_NOGAP_X);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("vxMapImagePatch() failed\n");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        // Copy Luma
        num_bytes = fwrite(data_ptr,1, img_width*img_height, fp);

        if (num_bytes != (img_width*img_height))
        {
            LOG_ERROR("Luma bytes read = %d, expected = %d\n",
                        num_bytes, img_width*img_height);

            vxStatus = VX_FAILURE;
        }

        vxUnmapImagePatch(image, map_id);
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        rect.start_x = 0;
        rect.start_y = 0;
        rect.end_x = img_width;
        rect.end_y = img_height / 2;
        vxStatus = vxMapImagePatch(image,
                                   &rect,
                                   1,
                                   &map_id,
                                   &image_addr,
                                   &data_ptr,
                                   VX_WRITE_ONLY,
                                   VX_MEMORY_TYPE_HOST,
                                   VX_NOGAP_X);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("vxMapImagePatch() failed\n");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        //Copy CbCr
        num_bytes = fwrite(data_ptr,1,img_width*img_height/2, fp);

        if (num_bytes != (img_width*img_height/2))
        {
            LOG_ERROR("CbCr bytes read = %d, expected = %d",
                      num_bytes, img_width*img_height/2);

            vxStatus = VX_FAILURE;
        }

        vxUnmapImagePatch(image, map_id);
    }

    if (fp != NULL)
    {
        fclose(fp);
    }

    return vxStatus;

}

vx_status CM_copyImage2Image(vx_image srcImage, vx_image dstImage)
{
    vx_map_id                  map_id;
    vx_rectangle_t             rect;
    vx_imagepatch_addressing_t  image_addr_1;
    vx_imagepatch_addressing_t  image_addr_2;
    vx_df_image                img_format;
    vx_uint32                  img_width;
    vx_uint32                  img_height;
    vx_status                  vxStatus;

    uint8_t                    *data_ptr_src_1;
    uint8_t                    *data_ptr_src_2;

    vxQueryImage(srcImage, VX_IMAGE_FORMAT, &img_format, sizeof(vx_df_image));
    vxQueryImage(srcImage, VX_IMAGE_WIDTH, &img_width, sizeof(vx_uint32));
    vxQueryImage(srcImage, VX_IMAGE_HEIGHT, &img_height, sizeof(vx_uint32));

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x   = img_width;
    rect.end_y   = img_height;

    // get source pointer
    vxStatus = vxMapImagePatch(srcImage,
                               &rect,
                               0,
                               &map_id,
                               &image_addr_1,
                               (void **)&data_ptr_src_1,
                               VX_READ_ONLY,
                               VX_MEMORY_TYPE_HOST,
                               VX_NOGAP_X);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("vxMapImagePatch() failed.");
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxUnmapImagePatch(srcImage, map_id);
        vxCopyImagePatch(dstImage, &rect, 0, &image_addr_1, data_ptr_src_1, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
    }

    // chroma
    if (img_format == VX_DF_IMAGE_NV12)
    {
        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            // get source pointer
            vxStatus = vxMapImagePatch(srcImage,
                                       &rect,
                                       1,
                                       &map_id,
                                       &image_addr_2,
                                       (void **)&data_ptr_src_2,
                                       VX_READ_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       VX_NOGAP_X);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("vxMapImagePatch() failed.");
            }
        }

        if (vxStatus == (vx_status)VX_SUCCESS)
        {
            vxUnmapImagePatch(srcImage, map_id);
            vxCopyImagePatch(dstImage, &rect, 1, &image_addr_2, data_ptr_src_2, VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);
        }
    }

    return vxStatus;
}


vx_status CM_copyData2Image(const uint8_t * data_ptr_src,
                            vx_image        dstImage,
                            int16_t         hSrcOfst,
                            int16_t         vSrcOfst)
{
    vx_map_id                  map_id;
    vx_uint32                  img_width;
    vx_uint32                  img_height;
    vx_df_image                img_format;
    vx_rectangle_t             rect;
    vx_imagepatch_addressing_t image_addr;
    int32_t                    j;
    int32_t                    widthInByte;
    uint8_t                  * data_ptr_dst;
    vx_status                  vxStatus;

    vxQueryImage(dstImage, VX_IMAGE_FORMAT, &img_format, sizeof(vx_df_image));
    vxQueryImage(dstImage, VX_IMAGE_WIDTH,  &img_width, sizeof(vx_uint32));
    vxQueryImage(dstImage, VX_IMAGE_HEIGHT, &img_height, sizeof(vx_uint32));

    rect.start_x = 0;
    rect.start_y = 0;
    rect.end_x   = img_width;
    rect.end_y   = img_height;

    // Get destination pointer
    vxStatus = vxMapImagePatch(dstImage,
                               &rect,
                               0,
                               &map_id,
                               &image_addr,
                               (void **)&data_ptr_dst,
                               VX_WRITE_ONLY,
                               VX_MEMORY_TYPE_HOST,
                               VX_NOGAP_X);

    if (vxStatus != (vx_status)VX_SUCCESS)
    {
        LOG_ERROR("vxMapImagePatch() failed.");
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        widthInByte = image_addr.dim_x * image_addr.stride_x;

        // Initialize the starting position from which the image data is copied
        data_ptr_src += vSrcOfst*widthInByte + hSrcOfst;

        for (j = 0; j < image_addr.dim_y; j++)
        {
            memcpy(data_ptr_dst, data_ptr_src, widthInByte);
            data_ptr_src += widthInByte;
            data_ptr_dst += image_addr.stride_y;
        }

        vxUnmapImagePatch(dstImage, map_id);
    }

    // chroma
    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        if (img_format == VX_DF_IMAGE_NV12)
        {
            rect.start_x = 0;
            rect.start_y = 0;
            rect.end_x   = img_width;
            rect.end_y   = img_height/2;

            // Get destination pointer
            vxStatus = vxMapImagePatch(dstImage,
                                       &rect,
                                       1,
                                       &map_id,
                                       &image_addr,
                                       (void **)&data_ptr_dst,
                                       VX_WRITE_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       VX_NOGAP_X);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("vxMapImagePatch() failed.");
            }
            else
            {
                widthInByte = image_addr.dim_x;

                // initialize the starting position from which the image data is copied
                data_ptr_src += (vSrcOfst*widthInByte/2) + hSrcOfst;

                for (j = 0; j < image_addr.dim_y; j++)
                {
                    memcpy(data_ptr_dst, data_ptr_src, widthInByte);
                    data_ptr_src += widthInByte;
                    data_ptr_dst += image_addr.stride_y;
                }

                vxUnmapImagePatch(dstImage, map_id);
            }
        }
    }

    return vxStatus;
}

vx_status CM_convertYUV2RGB(uint8_t        *rgbImage,
                            const uint8_t  *yuvImage[2],
                            int32_t         width,
                            int32_t         height)
{
    const uint8_t  *srcPtrY;
    const uint8_t  *srcPtrUV;
    uint8_t        *dstPtrR;
    uint8_t        *dstPtrG;
    uint8_t        *dstPtrB;
    int32_t         i;
    int32_t         j;
    int32_t         y;
    int32_t         cb;
    int32_t         cr;
    int32_t         r;
    int32_t         g;
    int32_t         b;
    vx_status       vxStatus;

    vxStatus = VX_SUCCESS;

    if (rgbImage == NULL)
    {
        LOG_ERROR("Parameter 'rgbImage' is NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (yuvImage[0] == NULL)
    {
        LOG_ERROR("Parameter 'yuvImage[0]' is NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (yuvImage[1] == NULL)
    {
        LOG_ERROR("Parameter 'yuvImage[1]' is NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (width == 0)
    {
        LOG_ERROR("Parameter 'width' cannot be 0.");
        vxStatus = VX_FAILURE;
    }
    else if (height == 0)
    {
        LOG_ERROR("Parameter 'height' cannot be 0.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        srcPtrY  = yuvImage[0];
        srcPtrUV = yuvImage[1];

        dstPtrR  = rgbImage;
        dstPtrG  = rgbImage + 1;
        dstPtrB  = rgbImage + 2;

        for (j = 0; j < height; j++)
        {
            for (i = 0; i < width; i++)
            {
                y  = srcPtrY[j * width + i];
                cb = srcPtrUV[(j >> 1)*width + (i>>1)*2];
                cr = srcPtrUV[(j >> 1)*width + (i>>1)*2 + 1];

                y  = y  - 16;
                cb = cb - 128;
                cr = cr - 128;

                r = clip3((298*y + 409*cr + 128) >> 8, 0, 255);
                g = clip3((298*y - 100*cb - 208*cr + 128) >> 8, 0, 255);
                b = clip3((298*y + 516*cb + 128) >> 8, 0, 255);

                *dstPtrR = (uint8_t)r; dstPtrR += 3;
                *dstPtrG = (uint8_t)g; dstPtrG += 3;
                *dstPtrB = (uint8_t)b; dstPtrB += 3;
            }
        }
    }

    return vxStatus;
}


vx_status CM_extractImageData(uint8_t         *outImageData,
                              const vx_image   yuvImage,
                              uint32_t         width,
                              uint32_t         height,
                              uint8_t          numPlanes,
                              uint8_t          rgbFlag)
{
    const uint8_t              *dataPtr[2];
    vx_imagepatch_addressing_t  addr;
    vx_df_image                 img_format;
    uint32_t                    i;
    vx_status                   vxStatus;

    vxStatus = VX_SUCCESS;

    if (outImageData == NULL)
    {
        LOG_ERROR("Parameter 'outImageData' is NULL.");
        vxStatus = VX_FAILURE;
    }
    else if (yuvImage == NULL)
    {
        LOG_ERROR("Parameter 'yuvImage' is NULL.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vxStatus = vxQueryImage(yuvImage,
                                VX_IMAGE_FORMAT,
                                &img_format,
                                sizeof(vx_df_image));

        if ((img_format != VX_DF_IMAGE_NV12) &&
            (img_format != VX_DF_IMAGE_S16)  &&
            (img_format != VX_DF_IMAGE_RGB))
        {
            LOG_ERROR("Image format is NOT supported.");
            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        vx_rectangle_t  rect;
        vx_map_id       mapId;

        // Define the ROI.
        rect.start_x = 0;
        rect.start_y = 0;
        rect.end_x   = width;
        rect.end_y   = height;

        for (i = 0; i < numPlanes; i++)
        {
            vxStatus = vxMapImagePatch(yuvImage,
                                       &rect,
                                       i,
                                       &mapId,
                                       &addr,
                                       (void **)&dataPtr[i],
                                       VX_READ_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       VX_NOGAP_X);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("vxMapImagePatch() failed.");
                break;
            }

            vxUnmapImagePatch(yuvImage, mapId);
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        if (rgbFlag == 0)
        {
            uint8_t    *dstImgPtr;
            uint32_t    size = 0;

            /* Copy the YUV data as is, Y followed by UV. */
            dstImgPtr = outImageData;

            if (img_format == VX_DF_IMAGE_NV12)
            {
                size = width * height;
            }
            else if (img_format == VX_DF_IMAGE_S16)
            {
                size = 2 * width * height;
            }
            else if (img_format == VX_DF_IMAGE_RGB)
            {
                size = 3 * width * height;
            }

            for (i = 0; i < numPlanes; i++, size /= 2)
            {
                memcpy(dstImgPtr, dataPtr[i], size);
                dstImgPtr += size;
            }
        }
        else
        {
            /* Convert to RGB data format. */
            vxStatus = CM_convertYUV2RGB(outImageData,
                                         dataPtr,
                                         width,
                                         height);

            if (vxStatus != (vx_status)VX_SUCCESS)
            {
                LOG_ERROR("CM_convertYUV2RGB() failed.");
                vxStatus = VX_FAILURE;
            }
        }
    }

    return vxStatus;
}

vx_status CM_extractTensorData(uint8_t         *outTensorData,
                               const vx_tensor  tensor,
                               uint32_t         size)
{
    uint8_t * output_buffer = NULL;
    vx_map_id map_id;
    vx_size   num_dims;
    vx_size   start[CM_MAX_TENSOR_DIMS];
    vx_size   tensor_strides[CM_MAX_TENSOR_DIMS];
    vx_size   tensor_sizes[CM_MAX_TENSOR_DIMS];
    vx_status vxStatus = VX_SUCCESS;

    memset(start, 0, sizeof(start));
    vxQueryTensor(tensor, VX_TENSOR_NUMBER_OF_DIMS, &num_dims, sizeof(vx_size));
    vxQueryTensor(tensor, VX_TENSOR_DIMS, tensor_sizes, num_dims * sizeof(vx_size));

    if (num_dims > CM_MAX_TENSOR_DIMS)
    {
        LOG_ERROR("Invalid number of dims read [%ld].", num_dims);
        vxStatus = VX_FAILURE;
    }
    else
    {
        vxStatus = tivxMapTensorPatch(tensor, num_dims, start, tensor_sizes,
                                      &map_id, tensor_strides,
                                      (void **)&output_buffer, VX_READ_ONLY,
                                      VX_MEMORY_TYPE_HOST);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("tivxMapTensorPatch() failed.");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        memcpy(outTensorData, output_buffer, size);
        vxStatus = tivxUnmapTensorPatch(tensor, map_id);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("tivxUnmapTensorPatch() failed.");
        }
    }

    return vxStatus;
}

#if !defined(SOC_AM62A) && !defined(SOC_AM62)
vx_status CM_extractPointCloudData(uint8_t                  *outPcData,
                                   const vx_user_data_object pointCloud,
                                   uint32_t                  pointSize,
                                   uint32_t                 *numPoints,
                                   uint8_t                   bCopyPoints)
{
    PTK_PointCloud  * pc;
    PTK_Point       * points;
    vx_map_id         mapId;
    vx_status         vxStatus = (vx_status) VX_SUCCESS;

    uint32_t          i;
    uint32_t          rgb;

    if (outPcData == NULL)
    {
        LOG_ERROR("Parameter 'outPcData' is NULL.");
        vxStatus = VX_FAILURE;
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Map the user object. */
        vxStatus = vxMapUserDataObject(pointCloud,
                                       0, // offset
                                       0, // size = 0 ==> entire buffer
                                       &mapId,
                                       (void **)&pc,
                                       VX_READ_ONLY,
                                       VX_MEMORY_TYPE_HOST,
                                       0);

        if ((vxStatus != (vx_status)VX_SUCCESS) || (pc == NULL))
        {
            LOG_ERROR("vxMapUserDataObject() failed.\n");

            vxStatus = VX_FAILURE;
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        /* Unmap the user object. */
        vxStatus = vxUnmapUserDataObject(pointCloud, mapId);

        if (vxStatus != (vx_status)VX_SUCCESS)
        {
            LOG_ERROR("vxUnmapUserDataObject() failed.\n");
        }
    }

    if (vxStatus == (vx_status)VX_SUCCESS)
    {
        *numPoints = pc->numPoints;

        if (bCopyPoints)
        {
            points     = PTK_PointCloud_getPoints(pc);

            for (i = 0; i < pc->numPoints; i++)
            {
                rgb = (uint32_t) points[i].meta.w;

                memcpy(&outPcData[i*pointSize    ], &points[i].x, sizeof(float));
                memcpy(&outPcData[i*pointSize + 4], &points[i].y, sizeof(float));
                memcpy(&outPcData[i*pointSize + 8], &points[i].z, sizeof(float));
                memcpy(&outPcData[i*pointSize + 12], &rgb, sizeof(uint32_t));
            }
        }
    }


    return vxStatus;
}
#endif // !defined(SOC_AM62A) && !defined(SOC_AM62)

vx_int32 CM_fill1DTensor(vx_tensor in_tensor, const vx_char* in_file)
{
    vx_status status = VX_SUCCESS;
    vx_map_id map_id_tensor;
    vx_uint8* tensor_buffer;

    FILE* fp;
    vx_int32 bytes_read = 0;

    vx_size    start[CM_MAX_TENSOR_DIMS];

    vx_size num_dims;
    vx_size size[3];
    vx_enum data_type;

    vxQueryTensor(in_tensor, VX_TENSOR_NUMBER_OF_DIMS, &num_dims, sizeof(num_dims));
    vxQueryTensor(in_tensor, VX_TENSOR_DIMS, &size, sizeof(vx_size)*3);
    vxQueryTensor(in_tensor, VX_TENSOR_DATA_TYPE, &data_type, sizeof(data_type));

    start[0]         = 0;
    start[1]         = 0;
    start[2]         = 0;

    vx_int32 tensor_size=1;
    for (int32_t i=0; i <  num_dims; i++){
        tensor_size *= size[i];
    }

    if ((data_type == VX_TYPE_FLOAT32) || (data_type == VX_TYPE_UINT32) || ((data_type == VX_TYPE_INT32)))
    {
        tensor_size *= sizeof(vx_int32);
    }

    if ((data_type == VX_TYPE_UINT16) || ((data_type == VX_TYPE_INT16)))
    {
        tensor_size *= sizeof(vx_int16);
    }

    status = tivxMapTensorPatch(in_tensor, num_dims, start, size, &map_id_tensor,
                                size,(void**) &tensor_buffer,
                                VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);

    if (status == VX_SUCCESS)
    {
        fp = fopen(in_file,"rb");
        if (fp == NULL)
        {
            LOG_ERROR("input binary file %s could not be opened \n", in_file);
        }
        else
        {
            fseek(fp,0L,SEEK_END);
            size[0] = ftell(fp);
            fseek(fp,0L,SEEK_SET);

            if (size[0] != tensor_size)
            {
                LOG_ERROR("Size of the tensor is lesser than binary file size\n");
            }
            else
            {
                bytes_read = fread(tensor_buffer, 1, size[0], fp);
            }
            fclose(fp);
      }

      tivxUnmapTensorPatch(in_tensor, map_id_tensor);
    }

    return (bytes_read);
}

vx_int32 CM_fill1DTensorFrom2Bin(vx_tensor in_tensor, const vx_char* in_file1, const vx_char* in_file2)
{
    vx_status status = VX_SUCCESS;
    vx_map_id map_id_tensor;
    vx_uint8* tensor_buffer;

    FILE* fp1;
    FILE* fp2;

    vx_int32 bytes_read = 0;

    vx_size    start[CM_MAX_TENSOR_DIMS];

    vx_size num_dims;
    vx_size size[3];
    vx_enum data_type;

    vxQueryTensor(in_tensor, VX_TENSOR_NUMBER_OF_DIMS, &num_dims, sizeof(num_dims));
    vxQueryTensor(in_tensor, VX_TENSOR_DIMS, &size, sizeof(vx_size)*3);
    vxQueryTensor(in_tensor, VX_TENSOR_DATA_TYPE, &data_type, sizeof(data_type));

    start[0]  = 0;
    start[1]  = 0;
    start[2]  = 0;

    vx_int32 tensor_size=1;

    for (int32_t i=0; i <  num_dims; i++)
    {
        tensor_size *= size[i];
    }

    if ((data_type == VX_TYPE_FLOAT32) || (data_type == VX_TYPE_UINT32) || ((data_type == VX_TYPE_INT32)))
    {
        tensor_size *= sizeof(vx_int32);
    }

    if ((data_type == VX_TYPE_UINT16) || ((data_type == VX_TYPE_INT16)))
    {
        tensor_size *= sizeof(vx_int16);
    }

    status = tivxMapTensorPatch(in_tensor, num_dims, start, size, &map_id_tensor,
                                size,(void**) &tensor_buffer,
                                VX_WRITE_ONLY, VX_MEMORY_TYPE_HOST);

    if (status == VX_SUCCESS)
    {
        fp1 = fopen(in_file1,"rb");
        fp2 = fopen(in_file2,"rb");

        if ((fp1 == NULL) && (fp2 == NULL))
        {
            LOG_ERROR("input binary file %s and %s could not be opened \n", in_file1, in_file2);
        }
        else if ((fp1 != NULL) && (fp2 == NULL))
        {
            LOG_ERROR("input binary file %s could not be opened \n", in_file1);
            fclose(fp1);
        }
        else if ((fp1 == NULL) && (fp2 != NULL))
        {
            LOG_ERROR("input binary file %s could not be opened \n", in_file2);
            fclose(fp2);
        }
        else
        {
            fseek(fp1,0L,SEEK_END);
            size[0] = ftell(fp1);
            fseek(fp1,0L,SEEK_SET);

            fseek(fp2,0L,SEEK_END);
            size[1] = ftell(fp2);
            fseek(fp2,0L,SEEK_SET);

            if ((size[0] + size[1])  != tensor_size)
            {
                LOG_ERROR("Size of the tensor is lesser than binary file size\n");
            }
            else
            {
                bytes_read  = fread(tensor_buffer, 1, size[0], fp1);
                bytes_read += fread(tensor_buffer+bytes_read, 1, size[1], fp2);
            }

            fclose(fp1);
            fclose(fp2);
        }

        tivxUnmapTensorPatch(in_tensor, map_id_tensor);
    }

    return (bytes_read);
}


vx_status CM_createDlBuffers(DLInferer            *inferer,
                             const VecDlTensor    *ifInfoList,
                             VecDlTensorPtr       &vecVar)
{
    vecVar.reserve(ifInfoList->size());

    for (int i = 0; i < ifInfoList->size(); i++)
    {
        const DlTensor *ifInfo = &ifInfoList->at(i);
        DlTensor   *obj = new DlTensor(*ifInfo);

        /* Allocate data buffer. */
        obj->allocateDataBuffer(*inferer);

        vecVar.push_back(obj);
    }

    return VX_SUCCESS;
}


} // namespace ti_core_common
