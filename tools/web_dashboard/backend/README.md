# ⚙️ LunaBot Web Ground Station — Backend Architecture

This directory houses the core server engine, ROS 2 DDS telemetry bridge, and AI reasoning pipelines for LunaBot Mission Control.

```
backend/
├── app.py              # Asynchronous FastAPI web server & background ROS 2 spin daemon
└── xai_copilot.py      # Non-rule-based Explainable AI semantic vector NLP engine & LLM integration
```

## Architectural Highlights

1. **Dual-Threaded ROS 2 + Async FastAPI Server**:
   - `start_ros_spin()` runs in a background daemon thread spinning `WebTelemetryNode` over ROS 2 Humble DDS.
   - FastAPI server handles high-throughput asynchronous HTTP/REST and MJPEG multipart video streaming on port 8080.

2. **Video & Sensor Streaming Pipelines**:
   - Thread-safe frame buffers (`_frame_lock`) delivering non-flickering multipart MJPEG streams for Left, Right, Rear, and SGBM Stereo 3D Depth feeds.

3. **Explainable AI (XAI) Natural Language Copilot**:
   - **Semantic Vector Space Model**: Uses Scikit-Learn TF-IDF N-gram tokenization and Cosine Similarity over an extensive domain corpus covering Apollo terramechanics, NASA LADEE lunar exosphere parameters, and rover kinematics.
   - **Optional LLM Integration**: Automatically calls Google Gemini 1.5 Flash when `GEMINI_API_KEY` is provided, grounding generative reasoning directly in real-time robot state vectors.

4. **Hardware Watchdog & Failsafe**:
   - Monitors physical Raspberry Pi 4B Edge Gateway heartbeats.
   - Triggers automated failsafe motor halt if communication is lost for >2.5 seconds.
