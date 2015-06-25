# Copyright: (C) 2015s iCub Facility - Istituto Italiano di Tecnologia
# Author: Raffaello Camoriano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname circonfArm)

file(GLOB templates ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml.template)
file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)
file(GLOB conf    ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

message (STATUS ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
message (STATUS ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR})
message (STATUS ${scripts})

### create a single target that installs all applications at once
yarp_install(FILES ${templates} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES ${conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${appname})

add_subdirectory(conf)