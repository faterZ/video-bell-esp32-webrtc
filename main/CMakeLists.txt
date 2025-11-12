idf_component_register(SRCS "webrtc.c"  "main.c" "board.c" "media_sys.c" "lcd_test.c"
                       EMBED_TXTFILES "ring.aac" "open.aac" "join.aac"
                       INCLUDE_DIRS "."
                       REQUIRES common BSP codec_board av_render esp_peer esp_webrtc 
                                media_lib_sal webrtc_utils esp_http_client esp_capture
                                espressif__esp_audio_codec espressif__esp_video_codec
                                console nvs_flash esp_wifi)
