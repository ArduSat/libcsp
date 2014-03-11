/*
	Licensed to the Apache Software Foundation (ASF) under one
	or more contributor license agreements.	 See the NOTICE file
	distributed with this work for additional information
	regarding copyright ownership.	The ASF licenses this file
	to you under the Apache License, Version 2.0 (the
	"License"); you may not use this file except in compliance
	with the License.	 You may obtain a copy of the License at

	http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing,
	software distributed under the License is distributed on an
	"AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
	KIND, either express or implied.	See the License for the
	specific language governing permissions and limitations
	under the License.
*/
/**
 * can_at91sam3x8e.h
 * Glue between SAM3X8E CAN drivers and CSP CAN interface
 * This includes prototypes available which do not belong to csp_if_can.h
 */

#ifndef _CAN_AT91SAM3X8E_H_
#define _CAN_AT91SAM3X8E_H_

#ifdef __cplusplus
extern "C" {
#endif

    int can_reset (int8_t index);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* _CAN_AT91SAM3X8E_H_ */
