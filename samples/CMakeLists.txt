if(DESKTOP_TEST)
  add_test(pcd_capture ${CMAKE_CURRENT_SOURCE_DIR}/ros/pcd_capture.py)

  add_test(pcd_display ${CMAKE_CURRENT_SOURCE_DIR}/standalone/pcd_display.py
    ${CMAKE_CURRENT_SOURCE_DIR}/still_life.pcd)
endif()

