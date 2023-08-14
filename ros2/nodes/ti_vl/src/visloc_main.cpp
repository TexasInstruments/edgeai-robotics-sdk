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
 * licensed and provided to you in appCntxtect code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *
 * *       any redistribution and use of the source code, including any resulting derivative
 * works, are licensed by TI for use only with TI Devices.
 *
 * *       any redistribution and use of any appCntxtect code compiled from the source code
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

#include <signal.h>

#include <visloc_node.h>

static std::shared_ptr<VisLocNode> visLocNode = nullptr;

static void sigHandler(int32_t sig)
{
    if (visLocNode)
    {
        visLocNode->sigHandler(sig);
    }

    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    try
    {
        rclcpp::InitOptions initOptions{};
        rclcpp::NodeOptions nodeOptions{};

        /* Prevent the RCLCPP signal handler binding. */
        initOptions.shutdown_on_signal = false;

        rclcpp::init(argc, argv, initOptions);

        signal(SIGINT, sigHandler);

        /* Allow any parameter name to be set on the node without first being
         * declared. Otherwise, setting an undeclared parameter will raise an
         * exception.
         *
         * This option being true does not affect parameter_overrides, as the
         * first set action will implicitly declare the parameter and therefore
         * consider any parameter overrides.
         */
        nodeOptions.allow_undeclared_parameters(true);

        /* Automatically iterate through the node's parameter overrides and
         * implicitly declare any that have not already been declared.
         * Otherwise, parameters passed to the node's parameter_overrides,
         * and/or the global arguments (e.g. parameter overrides from a YAML
         * file), which are not explicitly declared will not appear on the node
         * at all, even if allow_undeclared_parameters is true. Already
         * declared parameters will not be re-declared, and parameters declared
         * in this way will use the default constructed ParameterDescriptor.
         */
        nodeOptions.automatically_declare_parameters_from_overrides(true);

        /* Messages on topics which are published and subscribed to within this
         * context will go through a special intra-process communication code
         * code path which can avoid serialization and deserialization,
         * unnecessary copies, and achieve lower latencies in some cases.
         *
         * Defaults to false for now, as there are still some cases where it is
         * not desirable.
         */
        nodeOptions.use_intra_process_comms(false);

        visLocNode = std::make_shared<VisLocNode>(nodeOptions);

        rclcpp::spin(visLocNode);

        return EXIT_SUCCESS;
    }

    catch (std::runtime_error& e)
    {
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    return 0;
}

