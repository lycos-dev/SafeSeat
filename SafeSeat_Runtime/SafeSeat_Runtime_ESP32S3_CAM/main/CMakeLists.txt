idf_component_register(
    SRCS
        "app_main.cpp"
        "safeseat_pose_anomaly.cpp"
        "safeseat_occupant_anchor.cpp"
        "safeseat_temporal_filter.cpp"
        "frame_quality.cpp"
        "model_data.cpp"
    INCLUDE_DIRS "."
    REQUIRES coco_pose esp32-camera esp_wifi esp_event esp_netif nvs_flash
)
