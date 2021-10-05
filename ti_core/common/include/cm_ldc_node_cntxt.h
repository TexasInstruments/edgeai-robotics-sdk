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
#if !defined(_CM_LDC_NODE_CNTXT_H_)
#define _CM_LDC_NODE_CNTXT_H_

#include <cm_common.h>

/**
 * \defgroup group_ticore_node_ldc LDC node 
 * 
 * \brief Set up the LDC node
 * 
 * \ingroup group_ticore_node
 *
 */

namespace ti_core_common 
{

/**
 * \brief Constant for representing the ldc node context invalid state.
 * \ingroup group_ticore_node_ldc
 */
#define CM_LDC_NODE_CNTXT_STATE_INVALID     (0U)

/**
 * \brief Constant for representing the ldc node context initialization
 *        state.
 * \ingroup group_ticore_node_ldc
 */
#define CM_LDC_NODE_CNTXT_STATE_INIT        (1U)

/**
 * \brief Constant for representing the ldc node context setup state.
 * \ingroup group_ticore_node_ldc
 */
#define CM_LDC_NODE_CNTXT_STATE_SETUP       (2U)

/**
 * \brief Ldc node create time parameters.
 *
 * \ingroup group_ticore_node_ldc
 */
typedef struct
{
    /** Image width in pixels. */
    uint32_t                width;

    /** Image height in pixels. */
    uint32_t                height;

    /** Sub-sampling factor. */
    uint32_t                ssFactor;

    /** LDC block width. */
    uint32_t                blockWidth;

    /** LDC block height. */
    uint32_t                blockHeight;

    /** Pixed padding. */
    uint32_t                pixelPad;

    /** Path to the LUT definition file.*/
    char                   *lutFilePath;

} CM_LdcCreateParams;

/**
 * \brief Ldc node context.
 *
 * \ingroup group_ticore_node_ldc
 */
typedef struct CM_LdcNodeCntxt
{
    /** State variable. */
    uint32_t                state;

    /** Handle to the ldc node. */
    vx_node                 vxNode;

    /** Handle to the Mesh image. */
    vx_image                vxMeshImage;

    /** Handle to the output image of the ldc node. */
    vx_image                outImage;

    /** Handle to the data object holding the LDC configuration parameters. */
    vx_user_data_object     vxParamConfig;

    /** Handle to the data object holding the LDC mesh configuration
     * parameters.
     */
    vx_user_data_object     vxMeshConfig;

    /** Handle to the data object holding the LDC region configuration
     * parameters.
     */
    vx_user_data_object     vxRegionConfig;

} CM_LdcNodeCntxt;

/**
 * \brief Function to initialize the ldc node context.
 *
 * \param [in,out] ldcObj Ldc node context.
 *
 * \param [in] context The handle to the openVX implementation context.
 *
 * \param [in] createParams Ldc node context create parameters.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_node_ldc
 */
vx_status CM_ldcNodeCntxtInit(
        CM_LdcNodeCntxt            *ldcObj,
        vx_context                  context,
        const CM_LdcCreateParams   *createParams);

/**
 * \brief Function to create and setup the ldc node.
 *        
 *        The node should have been initialized by calling
 *        CM_ldcNodeCntxtInit() API, prior to invoking this API.
 *
 * \param [in,out] ldcObj Ldc node context.
 *
 * \param [in] context The handle to the openVX implementation context.
 *
 * \param [in] graph The handle to the graph this node belongs to.
 *
 * \param [in] inputImage Input image parameter to the node.

 * \param [in] outputIamge Output image parameter to the node.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_node_ldc
 */
vx_status CM_ldcNodeCntxtSetup(
        CM_LdcNodeCntxt    *ldcObj,
        vx_context          context,
        vx_graph            graph,
        vx_image            inputImage,
        vx_image            outputIamge);

/**
 * \brief Function to save the output image of the ldc node.
 *
 *        The node should have been initialized by calling
 *        CM_ldcNodeCntxtInit() API, prior to invoking this API.
 *
 * \param [in,out] ldcObj Ldc node context.
 *
 * \param [in] fileName Name of the image file to save to.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_node_ldc
 */
vx_status CM_ldcNodeCntxtSaveOutImage(
        CM_LdcNodeCntxt    *ldcObj,
        const char         *fileName);

/**
 * \brief Function to de-initialize the ldc node context.
 *
 * \param [in,out] ldcObj Ldc node context.
 *
 * \return VX_SUCCESS on success
 *
 * \ingroup group_ticore_node_ldc
 */
vx_status CM_ldcNodeCntxtDeInit(
        CM_LdcNodeCntxt    *ldcObj);

} // namespace ti_core_common 

#endif /* _CM_LDC_NODE_CNTXT_H_ */

