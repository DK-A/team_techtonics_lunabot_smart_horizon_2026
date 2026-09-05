// ── Clock ──
    function updateClock() {
      const now = new Date();
      document.getElementById('clock').innerText = now.toUTCString().replace("GMT", "UTC");
    }
    setInterval(updateClock, 1000); updateClock();

    // ── Target Dispatch Helper ──
    async function dispatchTarget(x, y, label) {
      if (window.isSimulationFrozen) {
        alert("❄️ ROVER SYSTEM FROZEN\n\nThe simulation runs ONLY when the physical Raspberry Pi 4B flight computer is online.\n\nRun 'python3 edge_pi/edge_agent.py' on the Pi terminal to unlock rover navigation!");
        return;
      }
      // 1. Automatically update the input fields with the target coordinates
      const inpX = document.getElementById('customX');
      const inpY = document.getElementById('customY');
      const numX = parseFloat(x);
      const numY = parseFloat(y);
      if (inpX) inpX.value = numX.toFixed(1);
      if (inpY) inpY.value = numY.toFixed(1);
      if (document.activeElement && ['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) {
        document.activeElement.blur();
      }

      const toast = document.getElementById('navToast');
      toast.innerText = `⏳ Sending Nav2 Goal [${label}]: (${numX.toFixed(1)}, ${numY.toFixed(1)})m ...`;
      toast.style.color = "#00e5ff";

      try {
        const res = await fetch('/api/send_goal', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({x: numX, y: numY, label: label})
        });
        const data = await res.json();
        if (data.success) {
          toast.innerText = `🚀 Nav2 Dispatched → ${label} (${numX.toFixed(1)}, ${numY.toFixed(1)})m`;
          toast.style.color = "#00e676";
          document.getElementById('v-nav-state').innerText = "NAVIGATING";
          document.getElementById('v-nav-target').innerText = `${label} (${numX.toFixed(1)}, ${numY.toFixed(1)})m`;
        } else {
          toast.innerText = `❌ Dispatch Error: ${data.error}`;
          toast.style.color = "#ff334b";
        }
      } catch(e) {
        toast.innerText = "❌ Network error dispatching goal";
        toast.style.color = "#ff334b";
      }
    }

    function dispatchCustomGoal() {
      const inpX = document.getElementById('customX');
      const inpY = document.getElementById('customY');
      const x = parseFloat(inpX ? inpX.value : 0.0) || 0.0;
      const y = parseFloat(inpY ? inpY.value : 0.0) || 0.0;
      if (document.activeElement) document.activeElement.blur();
      dispatchTarget(x, y, `Target (${x.toFixed(1)}, ${y.toFixed(1)})`);
    }

    async function abortNavigation() {
      const toast = document.getElementById('navToast');
      toast.innerText = "🛑 Aborting Nav2 Navigation Goal...";
      try {
        const res = await fetch('/api/abort_goal', {method: 'POST'});
        const data = await res.json();
        if (data.success) {
          toast.innerText = "🛑 Navigation Goal Aborted — Rover stopped";
          toast.style.color = "#ff334b";
        }
      } catch(e) {}
    }

    async function toggleMapView() {
      try {
        const res = await fetch('/api/toggle_map_view', {method: 'POST'});
        const data = await res.json();
        if (data.success) {
          const btn = document.getElementById('viewToggleBtn');
          if (btn) {
            btn.innerText = (data.map_view_mode === 'AUTO_ZOOM') ? '🔍 VIEW: AUTO-ZOOM' : '🗺️ VIEW: FULL MAP';
          }
        }
      } catch(e) {}
    }

    async function saveSlamMap() {
      const toast = document.getElementById('navToast');
      toast.innerText = "⏳ Saving SLAM map to disk...";
      toast.style.color = "#00e5ff";
      try {
        const res = await fetch('/api/save_map', {method: 'POST'});
        const data = await res.json();
        if (data.success) {
          toast.innerText = `💾 ${data.message}`;
          toast.style.color = "#00e676";
        } else {
          toast.innerText = `❌ Error saving map: ${data.error}`;
          toast.style.color = "#ff334b";
        }
      } catch (e) {
        toast.innerText = "❌ Network error saving map";
        toast.style.color = "#ff334b";
      }
    }

    // ── Simulation Pause / Resume ──
    let isPaused = false;
    async function toggleSimulation() {
      const btn = document.getElementById('simBtn');
      const msg = document.getElementById('simMsg');
      const targetState = !isPaused;
      if (window.isSimulationFrozen && !targetState) {
        alert("❄️ CANNOT RESUME SIMULATION\n\nSimulation runs ONLY when the physical Raspberry Pi 4B flight computer is online.\n\nRun 'python3 edge_pi/edge_agent.py' on the Pi terminal to unfreeze!");
        return;
      }
      btn.disabled = true;
      msg.innerText = targetState ? "Pausing simulation..." : "Resuming simulation...";

      try {
        const res = await fetch('/api/sim_control', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({pause: targetState})
        });
        const data = await res.json();
        if (data.success) {
          isPaused = targetState;
          if (isPaused) {
            btn.className = "paused";
            btn.innerText = "▶ RESUME SIMULATION";
            msg.innerText = "Simulation paused";
          } else {
            btn.className = "running";
            btn.innerText = "⏸ PAUSE SIMULATION";
            msg.innerText = "Gazebo physics active";
          }
        } else {
          msg.innerText = "Command failed: " + (data.error || "Unknown");
        }
      } catch (err) {
        msg.innerText = "Connection error to server";
      } finally {
        setTimeout(() => { btn.disabled = false; }, 800);
      }
    }

    // ── LunaBot XAI Natural Language Copilot Client Logic ──
    function quickAsk(text) {
      const inp = document.getElementById('xai-query-input');
      if (inp) {
        inp.value = text;
        askXAICopilot();
      }
    }

    async function askXAICopilot() {
      const inp = document.getElementById('xai-query-input');
      const btn = document.getElementById('btn-ask-xai');
      const card = document.getElementById('xai-ai-answer-card');
      const textEl = document.getElementById('xai-answer-text');
      const engEl = document.getElementById('xai-answer-engine');
      const timeEl = document.getElementById('xai-answer-time');

      if (!inp || !inp.value.trim()) return;
      const question = inp.value.trim();
      const geminiKey = localStorage.getItem('LUNABOT_GEMINI_API_KEY') || '';

      btn.disabled = true;
      btn.innerText = "⏳ THINKING...";
      card.style.display = "block";
      textEl.innerHTML = "<span style='color:var(--dim);'>Analyzing telemetry vectors &amp; domain knowledge...</span>";
      engEl.innerText = geminiKey ? "⚡ Google Gemini 1.5 Flash (Generative LLM)" : "🧠 Scikit-Learn Vector Space Embeddings";

      try {
        const res = await fetch('/api/xai_chat', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({ question: question, gemini_api_key: geminiKey })
        });
        const data = await res.json();
        if (data.success) {
          textEl.innerText = data.answer;
          engEl.innerText = data.engine || (geminiKey ? "⚡ Google Gemini LLM" : "🧠 Semantic Vector Space Model");
          timeEl.innerText = new Date().toLocaleTimeString();
        } else {
          textEl.innerText = "Error: " + (data.error || "Unable to process query.");
        }
      } catch (err) {
        textEl.innerText = "Communication error connecting to LunaBot XAI Copilot API.";
      } finally {
        btn.disabled = false;
        btn.innerText = "🚀 ASK COPILOT";
      }
    }

    function toggleGeminiKeyModal() {
      const current = localStorage.getItem('LUNABOT_GEMINI_API_KEY') || '';
      const key = prompt("Enter your Google Gemini API Key (optional - leave blank to use onboard Scikit-Learn Semantic Vector Model):", current);
      if (key !== null) {
        if (key.trim()) {
          localStorage.setItem('LUNABOT_GEMINI_API_KEY', key.trim());
          alert("✅ Google Gemini API Key saved! LunaBot XAI Copilot will now use Gemini 1.5 Flash generative reasoning.");
          const pill = document.getElementById('xai-engine-pill');
          if (pill) { pill.innerText = "Google Gemini 1.5 Flash (LLM)"; pill.style.borderColor = "#00e676"; pill.style.color = "#00e676"; }
        } else {
          localStorage.removeItem('LUNABOT_GEMINI_API_KEY');
          alert("Switched back to onboard Scikit-Learn Semantic Vector Space Model (Zero Internet Required).");
          const pill = document.getElementById('xai-engine-pill');
          if (pill) { pill.innerText = "Vector Space Semantic Embeddings"; pill.style.borderColor = "var(--cyan)"; pill.style.color = "var(--cyan)"; }
        }
      }
    }

    // ── Telemetry Polling (High-frequency, Zero-cache, Concurrency-guarded) ──
    let isTelemetryPolling = false;
    async function pollTelemetry() {
      if (isTelemetryPolling) return;
      isTelemetryPolling = true;
      try {
        const res = await fetch('/api/telemetry?t=' + Date.now(), {
          cache: 'no-store',
          headers: {
            'Cache-Control': 'no-cache, no-store, must-revalidate',
            'Pragma': 'no-cache'
          }
        });
        if (!res.ok) return;
        const d = await res.json();
        if (!d) return;

        // 1. Update Edge Computing Gateway Telemetry (Top Priority for instant UI reaction)
        try {
          if (d.edge_device) {
            const ed = d.edge_device;
            const elCard = document.getElementById('v-edge-card');
            const elBadge = document.getElementById('v-edge-badge');
            const elHealth = document.getElementById('v-edge-health');
            const elRole = document.getElementById('v-edge-role');
            const elInf = document.getElementById('v-edge-ml');
            const elLat = document.getElementById('v-edge-latency');
            const elPkts = document.getElementById('v-edge-packets');

            if (ed.online) {
              if (elBadge) {
                elBadge.innerText = '🟢 ' + (ed.status || 'CONNECTED (LIVE PI 4B)');
                elBadge.className = 'badge bg';
              }
              if (elCard) {
                elCard.style.borderLeft = '3px solid var(--green)';
                elCard.style.background = 'rgba(0, 230, 118, 0.06)';
              }
              if (elRole) {
                elRole.innerText = ed.role || 'Active Rover Onboard Computer & ML Gateway';
                elRole.className = 'val g';
                elRole.style.color = 'var(--cyan)';
              }
              if (elInf) {
                elInf.innerText = ed.inference || 'IsoForest + Terramechanics ML Active';
                elInf.className = 'val g';
              }
              if (elLat) {
                const latStr = (ed.latency_ms !== null && ed.latency_ms !== undefined) ? `${ed.latency_ms} ms` : '0.18 ms';
                elLat.innerText = `🟢 ${latStr} (Direct Ethernet Wire)`;
                elLat.className = 'val g';
                elLat.style.color = 'var(--green)';
              }
              if (elHealth) {
                elHealth.innerHTML = `<span style="color:var(--green); font-weight:700;">${ed.cpu_temp}</span> | RAM <span style="color:var(--cyan); font-weight:600;">${ed.ram_usage}</span> | <span style="color:var(--yellow); font-weight:600;">${ed.load}</span> Load`;
                elHealth.className = 'val';
              }
              if (elPkts) {
                elPkts.innerText = `🟢 Packet #${ed.packets_sent || 1} (Live 1 Hz stream)`;
                elPkts.className = 'val g';
                elPkts.style.color = 'var(--green)';
              }
            } else {
              if (elBadge) {
                elBadge.innerText = '❌ OFFLINE (Awaiting Pi Launch)';
                elBadge.className = 'badge br';
              }
              if (elCard) {
                elCard.style.borderLeft = '3px solid var(--red)';
                elCard.style.background = 'rgba(255, 51, 75, 0.08)';
              }
              if (elRole) {
                elRole.innerText = 'Offline — Run python3 edge_agent.py on Pi';
                elRole.className = 'val r';
                elRole.style.color = 'var(--red)';
              }
              if (elInf) {
                elInf.innerText = 'OFFLINE (Standby)';
                elInf.className = 'val r';
              }
              if (elLat) {
                elLat.innerText = 'NO LINK (Disconnected)';
                elLat.className = 'val r';
                elLat.style.color = 'var(--red)';
              }
              if (elHealth) {
                elHealth.innerHTML = `<span style="color:var(--red); font-weight:bold;">⚠️ NO HEARTBEAT (OFFLINE)</span>`;
                elHealth.className = 'val r';
              }
              if (elPkts) {
                elPkts.innerText = '0 packets (Awaiting Pi start)';
                elPkts.className = 'val r';
                elPkts.style.color = 'var(--dim)';
              }
            }

            // Update Header Status Pill
            const hPill = document.getElementById('v-header-pill');
            if (hPill) {
              if (ed.online) {
                hPill.innerHTML = '<div class="dot"></div>TELEMETRY LIVE • PI 4B ONLINE';
                hPill.className = 'live-pill';
                hPill.style.background = 'rgba(0, 230, 118, 0.12)';
                hPill.style.color = 'var(--green)';
              } else {
                hPill.innerHTML = '<div class="dot" style="background:var(--red); box-shadow:0 0 8px var(--red);"></div>❌ PI 4B OFFLINE (SIM FROZEN)';
                hPill.className = 'live-pill';
                hPill.style.background = 'rgba(255, 51, 75, 0.20)';
                hPill.style.color = 'var(--red)';
              }
            }
          }
        } catch(eEdge) { console.warn("Edge render error:", eEdge); }

        // 2. Dynamic Simulation Freeze & Resume Control (Instant visual feedback)
        try {
          const isFrozen = (d.edge_device && d.edge_device.online) ? false : ((d.simulation_frozen !== undefined) ? !!d.simulation_frozen : false);
          window.isSimulationFrozen = isFrozen;

          const freezeBanner = document.getElementById('v-freeze-banner');
          const freezeReason = document.getElementById('v-freeze-reason');
          if (freezeBanner) {
            freezeBanner.style.display = isFrozen ? 'flex' : 'none';
          }
          if (freezeReason && d.freeze_reason) {
            freezeReason.innerText = d.freeze_reason;
          }

          // Visual cue on map and camera feeds
          const feeds = document.querySelectorAll('img.feed');
          feeds.forEach(f => {
            f.style.filter = isFrozen ? 'grayscale(45%) contrast(0.85) brightness(0.85)' : 'none';
          });

          // Disable/Enable Nav Buttons
          const navBtns = document.querySelectorAll('.btn-target, .btn-abort');
          navBtns.forEach(b => {
            if (isFrozen) {
              b.style.opacity = '0.45';
              b.style.cursor = 'not-allowed';
            } else {
              b.style.opacity = '1.0';
              b.style.cursor = 'pointer';
            }
          });

          // Update Simulation Pause/Resume button
          const simBtn = document.getElementById('simBtn');
          const simMsg = document.getElementById('simMsg');
          if (simBtn) {
            if (isFrozen) {
              simBtn.className = "paused";
              simBtn.innerText = "❄️ SIMULATION FROZEN (PI OFFLINE)";
              if (simMsg) simMsg.innerHTML = '<span style="color:var(--red); font-weight:700;">⚠️ Simulation frozen — Run edge_agent.py on Pi to unlock</span>';
            } else {
              simBtn.className = "running";
              simBtn.innerText = "⏸ PAUSE SIMULATION";
              if (simMsg) simMsg.innerHTML = '<span style="color:var(--green);">Gazebo physics active &bull; Pi 4B connected</span>';
            }
          }
        } catch(eFreeze) { console.warn("Freeze render error:", eFreeze); }

        // 3. Render Explainable AI (XAI) Live Decision Feed
        try {
          if (d.xai_logs && d.xai_logs.length > 0) {
            const feed = document.getElementById('xai-feed');
            if (feed) {
              feed.innerHTML = d.xai_logs.map(log => {
                let catColor = '#00e5ff';
                let badgeCls = 'bc';
                if (log.category === 'SCIENCE') { catColor = '#00e676'; badgeCls = 'bg'; }
                else if (log.category === 'TERRA') { catColor = '#ffd600'; badgeCls = 'by'; }
                else if (log.category === 'SAFETY') { catColor = '#ff334b'; badgeCls = 'br'; }
                else if (log.category === 'MISSION') { catColor = '#b388ff'; badgeCls = 'bb'; }
                else if (log.category === 'EDGE') { catColor = '#e040fb'; badgeCls = 'bm'; }
                else if (log.category === 'AI_COPILOT') { catColor = '#00e5ff'; badgeCls = 'bc'; }

                let textCol = log.severity === 'CRITICAL' ? '#ff5252' : (log.severity === 'WARN' ? '#ffd740' : (log.severity === 'SUCCESS' ? '#69f0ae' : '#eceff1'));
                return `<div style="display:flex; gap:8px; align-items:flex-start; padding:3px 6px; background:rgba(255,255,255,0.02); border-radius:4px; border-left:3px solid ${catColor};">
                  <span style="color:var(--dim); min-width:55px;">[${log.time}]</span>
                  <span class="badge ${badgeCls}" style="font-size:0.62rem; padding:1px 5px;">${log.category}</span>
                  <span style="color:${textCol}; flex:1;">${log.explanation}</span>
                </div>`;
              }).join('');
            }
          }
        } catch (eXai) { console.warn("XAI render error:", eXai); }

        // 4. Render Autonomous Patrol Button State
        try {
          if (d.patrol_active !== undefined) {
            const btn = document.getElementById('btn-patrol');
            if (btn) {
              isPatrolRunning = d.patrol_active;
              if (d.patrol_active) {
                btn.innerText = `🛑 STOP PATROL (WP #${d.patrol_index + 1})`;
                btn.style.background = 'rgba(255, 51, 75, 0.22)';
                btn.style.borderColor = 'var(--red)';
                btn.style.color = 'var(--red)';
              } else {
                btn.innerText = '🚀 START AUTONOMOUS PATROL';
                btn.style.background = 'rgba(0, 230, 118, 0.16)';
                btn.style.borderColor = 'var(--green)';
                btn.style.color = 'var(--green)';
              }
            }
          }
        } catch (ePat) {}

        // Render Obstacle Demo Button State
        try {
          if (d.obstacle_demo_active !== undefined) {
            const btnObs = document.getElementById('btn-obstacle-demo');
            if (btnObs) {
              isObstacleDemoRunning = d.obstacle_demo_active;
              if (d.obstacle_demo_active) {
                btnObs.innerText = '🛑 STOP OBSTACLE DEMO';
                btnObs.style.background = 'rgba(255, 51, 75, 0.22)';
                btnObs.style.borderColor = 'var(--red)';
                btnObs.style.color = 'var(--red)';
              } else {
                btnObs.innerText = '🛡️ OBSTACLE PASS-OVER DEMO';
                btnObs.style.background = 'rgba(0, 229, 255, 0.16)';
                btnObs.style.borderColor = 'var(--cyan)';
                btnObs.style.color = 'var(--cyan)';
              }
            }
          }
        } catch (eObs) {}

        // 5. Update Robot Kinematics & Mission Status
        try {
          if (d.robot_pose) {
            const elX = document.getElementById('v-x'); if (elX) elX.innerText = d.robot_pose.x.toFixed(2) + ' m';
            const elY = document.getElementById('v-y'); if (elY) elY.innerText = d.robot_pose.y.toFixed(2) + ' m';
          } else if (d.odom) {
            const elX = document.getElementById('v-x'); if (elX) elX.innerText = d.odom.x.toFixed(2) + ' m';
            const elY = document.getElementById('v-y'); if (elY) elY.innerText = d.odom.y.toFixed(2) + ' m';
          }
          if (d.odom) {
            const elSpd = document.getElementById('v-speed'); if (elSpd) elSpd.innerText = d.odom.speed.toFixed(2) + ' m/s';
          }
          if (d.imu) {
            const elGrav = document.getElementById('v-grav');
            if (elGrav) {
              const p = (d.imu.pitch !== undefined) ? ` | P:${d.imu.pitch.toFixed(1)}°` : '';
              elGrav.innerText = `${d.imu.acc_z.toFixed(3)} m/s²${p}`;
            }
          }
          if (d.nav_status) {
            const elSt = document.getElementById('v-nav-state'); if (elSt) elSt.innerText = d.nav_status;
            const elBadge = document.getElementById('v-nav-badge');
            if (elBadge) {
              elBadge.innerText = d.nav_status;
              elBadge.className = 'badge ' + (d.nav_status === 'NAVIGATING' ? 'bc' : (d.nav_status === 'TARGET_REACHED' ? 'bg' : (d.nav_status === 'IDLE' ? 'by' : 'br')));
            }
          }
          if (d.mission_activity) {
            const elAct = document.getElementById('v-activity');
            if (elAct) elAct.innerText = d.mission_activity;
          }
          const elTgt = document.getElementById('v-nav-target');
          if (elTgt) {
            elTgt.innerText = (d.current_target && d.current_target[2]) ? d.current_target[2] : "None";
          }
          if (d.distance_remaining !== undefined) {
            const elDist = document.getElementById('v-nav-dist');
            if (elDist) elDist.innerText = Number(d.distance_remaining).toFixed(2) + ' m';
          }
        } catch(eKin) {}

        // 6. Update Environmental Telemetry
        try {
          if (d.env) {
            if (d.env.environment_state) {
              const el = document.getElementById('v-atm-state');
              if (el) el.innerText = d.env.environment_state.replace('_', ' ');
            }
            if (d.env.pressure_display) {
              const el = document.getElementById('v-pres'); if (el) el.innerText = d.env.pressure_display;
            } else if (d.env.pressure_bmp390_hpa !== undefined || d.env.pressure_hpa !== undefined) {
              const p = Number(d.env.pressure_bmp390_hpa || d.env.pressure_hpa);
              const el = document.getElementById('v-pres');
              if (el) el.innerText = p < 0.001 ? p.toExponential(2) + ' hPa' : `${p.toFixed(2)} hPa`;
            }
            if (d.env.o2_percent !== undefined) {
              const o2Val = Number(d.env.o2_percent);
              const el = document.getElementById('v-o2');
              if (el) el.innerText = o2Val <= 0.01 ? '0.00 % (Vacuum)' : `${o2Val.toFixed(2)} %`;
            }
            if (d.env.ambient_temp_k !== undefined) {
              const c = (d.env.ambient_temp_k - 273.15).toFixed(1);
              const el = document.getElementById('v-temp');
              if (el) el.innerText = `${c} °C (${Number(d.env.ambient_temp_k).toFixed(1)} K)`;
            }
            if (d.env.dust_concentration_ug_m3 !== undefined) {
              const el = document.getElementById('v-dust');
              if (el) el.innerText = `${Number(d.env.dust_concentration_ug_m3).toFixed(1)} µg/m³`;
            }
            if (d.env.radiation_msv_h !== undefined) {
              const el = document.getElementById('v-rad');
              if (el) el.innerText = `${Number(d.env.radiation_msv_h).toFixed(3)} mSv/h`;
            }
            if (d.env.solar_flux_w_m2 !== undefined) {
              const el = document.getElementById('v-solar');
              if (el) el.innerText = `${Number(d.env.solar_flux_w_m2).toFixed(1)} W/m²`;
            }
            if (d.env.ml_anomaly_score !== undefined) {
              const el = document.getElementById('v-iso-anomaly');
              if (el) {
                const s = Number(d.env.ml_anomaly_score);
                el.innerText = `${s.toFixed(3)} (${d.env.ml_anomaly_detected ? 'ANOMALY' : 'NOMINAL'})`;
                el.className = 'val ' + (d.env.ml_anomaly_detected ? 'r' : 'g');
              }
            }
          }
        } catch(eEnv) {}

        // 7. Update Terramechanics
        try {
          if (d.terramechanics) {
            const tm = d.terramechanics;
            const slipPct = (tm.slip_ratio * 100).toFixed(1);
            const elSlip = document.getElementById('v-terra-slip'); if (elSlip) elSlip.innerText = `${slipPct} %`;
            const bar = document.getElementById('v-terra-slip-bar');
            if (bar) {
              bar.style.width = `${Math.min(100, Math.max(3, tm.slip_ratio * 100))}%`;
              bar.style.background = tm.slip_ratio > 0.5 ? 'var(--red)' : (tm.slip_ratio > 0.25 ? 'var(--orange)' : 'var(--green)');
            }
            const elSink = document.getElementById('v-terra-sinkage'); if (elSink) elSink.innerText = `${Number(tm.sinkage_mm).toFixed(1)} mm`;
            const elTrac = document.getElementById('v-terra-traction'); if (elTrac) elTrac.innerText = `${(tm.traction_coeff * 100).toFixed(0)} %`;
            const elAnom = document.getElementById('v-terra-anomaly'); if (elAnom) elAnom.innerText = `${Number(tm.anomaly_score).toFixed(2)}`;
            const badge = document.getElementById('v-terra-badge');
            if (badge) {
              badge.innerText = tm.anomaly_state || 'NOMINAL';
              badge.className = 'badge ' + (tm.anomaly_state === 'NOMINAL' ? 'bg' : (tm.anomaly_state === 'MODERATE_SLIP' ? 'by' : 'br'));
            }
          }
        } catch(eTm) {}

      } catch(e) {
        console.warn("pollTelemetry general exception:", e);
      } finally {
        isTelemetryPolling = false;
      }
    }
    pollTelemetry();
    setInterval(pollTelemetry, 200);

    let isPatrolRunning = false;
    async function toggleAutonomousPatrol() {
      if (window.isSimulationFrozen && !isPatrolRunning) {
        alert("❄️ ROVER SYSTEM FROZEN\n\nAutonomous Patrol requires the physical Raspberry Pi 4B flight computer.\n\nRun 'python3 edge_pi/edge_agent.py' on the Pi terminal to activate.");
        return;
      }
      const endpoint = isPatrolRunning ? '/api/patrol/stop' : '/api/patrol/start';
      try {
        const res = await fetch(endpoint, { method: 'POST' });
        const data = await res.json();
        if (data.success) {
          isPatrolRunning = data.patrol_active;
          const toast = document.getElementById('navToast');
          if (toast) {
            toast.innerText = isPatrolRunning ? "🚀 Autonomous Patrol Activated — Continuous Habitat Loop" : "🛑 Autonomous Patrol Stopped";
          }
        }
      } catch (e) {
        console.error("Patrol toggle error:", e);
      }
    }

    let isObstacleDemoRunning = false;
    async function toggleObstacleDemo() {
      if (window.isSimulationFrozen && !isObstacleDemoRunning) {
        alert("❄️ ROVER SYSTEM FROZEN\n\nObstacle Avoidance Demo requires the physical Raspberry Pi 4B flight computer.\n\nRun 'python3 edge_pi/edge_agent.py' on the Pi terminal to activate.");
        return;
      }
      const endpoint = isObstacleDemoRunning ? '/api/obstacle_demo/stop' : '/api/obstacle_demo/start';
      try {
        const res = await fetch(endpoint, { method: 'POST' });
        const data = await res.json();
        if (data.success) {
          isObstacleDemoRunning = data.obstacle_demo_active;
          const toast = document.getElementById('navToast');
          if (toast) {
            toast.innerText = isObstacleDemoRunning ? "🛡️ Obstacle Pass-Over Demo Activated — Generating Tangential Avoidance Arc" : "🛑 Obstacle Demo Stopped";
          }
        } else if (data.error) {
          alert("Obstacle Demo Error: " + data.error);
        }
      } catch (e) {
        console.error("Obstacle demo toggle error:", e);
      }
    }

    // ── Click Map → Dispatch Nav2 Waypoint ──
    const mapImg = document.getElementById('mapStream');
    mapImg.addEventListener('click', async (e) => {
      if (window.isSimulationFrozen) {
        alert("❄️ ROVER SYSTEM FROZEN\n\nThe simulation runs ONLY when the physical Raspberry Pi 4B flight computer is online.\n\nRun 'python3 edge_pi/edge_agent.py' on the Pi terminal to unlock click-to-navigate!");
        return;
      }
      const rect = mapImg.getBoundingClientRect();
      const normX = (e.clientX - rect.left) / rect.width;
      const normY = (e.clientY - rect.top) / rect.height;

      const toast = document.getElementById('navToast');
      try {
        const tr = await fetch('/api/telemetry');
        const td = await tr.json();
        if (!td.map_meta || !td.map_meta.width) {
          toast.innerText = "⚠️ Map initializing — please wait 2 seconds...";
          return;
        }

        let wx = 0, wy = 0;
        const vp = td.viewport;
        if (vp && vp.mode === 'AUTO_ZOOM' && vp.min_wx !== undefined) {
          wx = vp.min_wx + normX * (vp.max_wx - vp.min_wx);
          wy = vp.max_wy - normY * (vp.max_wy - vp.min_wy);
        } else {
          const m = td.map_meta;
          wx = m.origin_x + normX * m.width * m.resolution;
          wy = m.origin_y + (1.0 - normY) * m.height * m.resolution;
        }

        dispatchTarget(wx.toFixed(2), wy.toFixed(2), `Target (${wx.toFixed(1)}, ${wy.toFixed(1)})`);
      } catch(e) {}
    });

    // ── Auto-Reconnect Flaky Browser Streams ──
    document.querySelectorAll('img.feed').forEach(img => {
      img.onerror = () => {
        setTimeout(() => {
          const baseSrc = img.src.split('?')[0];
          img.src = baseSrc + '?t=' + Date.now();
        }, 1000);
      };
    });

    // ── Silent Keyboard Listener ──
    let activeKeys = {};
    let teleopTimer = null;

    function getNormalizedKey(e) {
      const code = e.code || '';
      const key = (e.key || '').toLowerCase();
      if (code === 'KeyW' || code === 'ArrowUp' || key === 'w' || key === 'arrowup') return 'w';
      if (code === 'KeyS' || code === 'ArrowDown' || key === 's' || key === 'arrowdown') return 's';
      if (code === 'KeyA' || code === 'ArrowLeft' || key === 'a' || key === 'arrowleft') return 'a';
      if (code === 'KeyD' || code === 'ArrowRight' || key === 'd' || key === 'arrowright') return 'd';
      if (code === 'Space' || key === ' ') return 'stop';
      return null;
    }

    // Auto-blur inputs when clicking anywhere outside an input
    document.addEventListener('click', (e) => {
      if (!['INPUT', 'TEXTAREA'].includes(e.target.tagName)) {
        if (document.activeElement && ['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) {
          document.activeElement.blur();
        }
      }
    });

    window.addEventListener('keydown', (e) => {
      if (['INPUT', 'TEXTAREA'].includes(document.activeElement?.tagName)) {
        if (e.key === 'Escape' || e.key === 'Enter') {
          document.activeElement.blur();
        }
        return;
      }
      const k = getNormalizedKey(e);
      if (k) {
        e.preventDefault();
        if (k === 'stop') {
          activeKeys = {};
          if (teleopTimer) { clearInterval(teleopTimer); teleopTimer = null; }
          sendTeleopStop();
          return;
        }
        activeKeys[k] = true;
        if (!teleopTimer) {
          sendTeleopStep();
          teleopTimer = setInterval(sendTeleopStep, 60);
        }
      }
    });

    window.addEventListener('keyup', (e) => {
      if (['INPUT', 'TEXTAREA'].includes(document.activeElement?.tagName)) return;
      const k = getNormalizedKey(e);
      if (k && k !== 'stop') {
        delete activeKeys[k];
        if (Object.keys(activeKeys).length === 0) {
          if (teleopTimer) { clearInterval(teleopTimer); teleopTimer = null; }
          sendTeleopStop();
        }
      }
    });

    window.addEventListener('blur', () => {
      activeKeys = {};
      if (teleopTimer) { clearInterval(teleopTimer); teleopTimer = null; }
      sendTeleopStop();
    });

    function sendTeleopStep() {
      let vx = 0.0;
      let wz = 0.0;
      if (activeKeys['w']) vx += 0.65;
      if (activeKeys['s']) vx -= 0.65;
      if (activeKeys['a']) wz += 0.75;
      if (activeKeys['d']) wz -= 0.75;

      try {
        fetch('/api/teleop', {
          method: 'POST',
          keepalive: true,
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({vx: vx, wz: wz})
        });
      } catch(e) {}
    }

    function sendTeleopStop() {
      try {
        fetch('/api/teleop', {
          method: 'POST',
          keepalive: true,
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({vx: 0.0, wz: 0.0})
        });
      } catch(e) {}
    }

    // ══════════════════════════════════════════════════════════════════════
    // MISSION RECORDING CONTROLLER (GAZEBO + DASHBOARD DUAL RECORDER)
    // ══════════════════════════════════════════════════════════════════════
    let isRecordingActive = false;
    let recTimerInterval = null;
    let recStartTime = 0;
    let browserMediaRecorder = null;
    let browserRecordedChunks = [];

    async function toggleMissionRecording() {
      const mode = document.getElementById('recMode').value;
      if (!isRecordingActive) {
        if (mode === 'desktop') {
          await startServerDesktopRecording();
        } else {
          await startBrowserMediaRecording();
        }
      } else {
        if (mode === 'desktop' || !browserMediaRecorder) {
          await stopServerDesktopRecording();
        } else {
          stopBrowserMediaRecording();
        }
      }
    }

    async function startServerDesktopRecording() {
      try {
        const res = await fetch('/api/record/start', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({label: 'mission'})
        });
        const data = await res.json();
        if (data.success) {
          setRecordingUI(true);
          showToast('🔴 RECORDING ACTIVE: Capturing Gazebo & Mission Control screen (1920x1080)');
        } else {
          alert('Recording failed to start: ' + (data.error || 'Unknown error'));
        }
      } catch (err) {
        alert('Failed to connect to recorder: ' + err.message);
      }
    }

    async function stopServerDesktopRecording() {
      try {
        showToast('⏳ Finalizing video recording...');
        const res = await fetch('/api/record/stop', {method: 'POST'});
        const data = await res.json();
        setRecordingUI(false);
        if (data.success && data.recording) {
          const rec = data.recording;
          showToast(`✅ Mission Recorded! ${rec.duration}s (${rec.size_mb} MB)`);
          openRecordingsModal();
          playMissionVideo(rec.url, rec.filename);
        } else {
          alert('Stop recording returned: ' + (data.error || 'File saved'));
        }
      } catch (err) {
        setRecordingUI(false);
        alert('Error stopping recording: ' + err.message);
      }
    }

    async function startBrowserMediaRecording() {
      try {
        const stream = await navigator.mediaDevices.getDisplayMedia({
          video: { cursor: "always", displaySurface: "monitor" },
          audio: false
        });
        browserRecordedChunks = [];
        const mime = MediaRecorder.isTypeSupported('video/webm;codecs=vp9') ? 'video/webm;codecs=vp9' : 'video/webm';
        browserMediaRecorder = new MediaRecorder(stream, { mimeType: mime });
        browserMediaRecorder.ondataavailable = (e) => {
          if (e.data && e.data.size > 0) browserRecordedChunks.push(e.data);
        };
        browserMediaRecorder.onstop = () => {
          const blob = new Blob(browserRecordedChunks, { type: mime });
          const url = URL.createObjectURL(blob);
          const a = document.createElement('a');
          const ts = new Date().toISOString().replace(/[:.]/g, '-');
          a.href = url;
          a.download = `lunabot_browser_record_${ts}.webm`;
          document.body.appendChild(a);
          a.click();
          setTimeout(() => { document.body.removeChild(a); URL.revokeObjectURL(url); }, 1000);
          setRecordingUI(false);
          showToast('✅ Browser screen recording downloaded!');
        };
        stream.getVideoTracks()[0].onended = () => {
          if (isRecordingActive) stopBrowserMediaRecording();
        };
        browserMediaRecorder.start(500);
        setRecordingUI(true);
        showToast('🔴 Browser Recording Started!');
      } catch (err) {
        alert('Display Media capture was canceled or not supported: ' + err.message);
        setRecordingUI(false);
      }
    }

    function stopBrowserMediaRecording() {
      if (browserMediaRecorder && browserMediaRecorder.state !== 'inactive') {
        browserMediaRecorder.stop();
        if (browserMediaRecorder.stream) {
          browserMediaRecorder.stream.getTracks().forEach(t => t.stop());
        }
      }
      setRecordingUI(false);
    }

    function setRecordingUI(active) {
      isRecordingActive = active;
      const btn = document.getElementById('recBtn');
      const txt = document.getElementById('recBtnText');
      const timer = document.getElementById('recTimer');
      if (active) {
        btn.classList.add('btn-recording');
        txt.innerText = '⏹️ STOP RECORDING';
        timer.style.display = 'inline-block';
        timer.innerText = '00:00';
        recStartTime = Date.now();
        if (recTimerInterval) clearInterval(recTimerInterval);
        recTimerInterval = setInterval(updateRecTimer, 1000);
      } else {
        btn.classList.remove('btn-recording');
        txt.innerText = '⏺️ RECORD MISSION';
        timer.style.display = 'none';
        if (recTimerInterval) {
          clearInterval(recTimerInterval);
          recTimerInterval = null;
        }
      }
    }

    function updateRecTimer() {
      const elapsed = Math.floor((Date.now() - recStartTime) / 1000);
      const m = String(Math.floor(elapsed / 60)).padStart(2, '0');
      const s = String(elapsed % 60).padStart(2, '0');
      const timer = document.getElementById('recTimer');
      if (timer) timer.innerText = `${m}:${s}`;
    }

    function openRecordingsModal() {
      const m = document.getElementById('recordingsModal');
      if (m) {
        m.style.display = 'flex';
        loadRecordingsList();
      }
    }

    function closeRecordingsModal() {
      const m = document.getElementById('recordingsModal');
      if (m) m.style.display = 'none';
      closeVideoPlayer();
    }

    function closeVideoPlayer() {
      const p = document.getElementById('missionVideoPlayer');
      if (p) { p.pause(); p.src = ''; }
      const c = document.getElementById('videoPlayerContainer');
      if (c) c.style.display = 'none';
    }

    function playMissionVideo(url, title) {
      const c = document.getElementById('videoPlayerContainer');
      const p = document.getElementById('missionVideoPlayer');
      const t = document.getElementById('videoPlayerTitle');
      if (c && p) {
        c.style.display = 'block';
        p.src = url;
        p.play();
        if (t) t.innerText = `▶️ ${title || 'Mission Recording'}`;
      }
    }

    async function loadRecordingsList() {
      const container = document.getElementById('recordingsListContent');
      if (!container) return;
      container.innerHTML = '<div style="text-align:center; color:var(--dim); padding:20px;">Fetching recordings...</div>';
      try {
        const res = await fetch('/api/recordings');
        const data = await res.json();
        if (!data.recordings || data.recordings.length === 0) {
          container.innerHTML = '<div style="text-align:center; color:var(--dim); padding:30px; font-size:0.9rem;">No recordings saved yet. Click <b>⏺️ RECORD MISSION</b> to record Gazebo and the Dashboard!</div>';
          return;
        }
        let html = '<table style="width:100%; border-collapse:collapse; font-size:0.8rem;">';
        html += '<tr style="border-bottom:1px solid #222d3d; color:var(--dim); text-align:left;"><th style="padding:8px;">File</th><th>Created</th><th>Size</th><th style="text-align:right;">Actions</th></tr>';
        data.recordings.forEach(r => {
          html += `<tr style="border-bottom:1px solid rgba(255,255,255,0.05);">
            <td style="padding:10px 8px; font-weight:600; color:#e6edf3;">${r.filename}</td>
            <td style="color:var(--dim);">${r.created_at}</td>
            <td style="color:var(--cyan); font-family:monospace;">${r.size_mb} MB</td>
            <td style="text-align:right;">
              <button onclick="playMissionVideo('${r.url}', '${r.filename}')" style="background:rgba(0,229,255,0.15); color:var(--cyan); border:1px solid var(--cyan); border-radius:4px; padding:3px 8px; font-size:0.75rem; cursor:pointer; margin-right:4px;">▶️ Play</button>
              <a href="${r.url}" download="${r.filename}" style="background:rgba(0,230,118,0.15); color:#00e676; border:1px solid #00e676; border-radius:4px; padding:3px 8px; font-size:0.75rem; text-decoration:none; display:inline-block; margin-right:4px;">⬇️ Download</a>
              <button onclick="deleteMissionRecording('${r.filename}')" style="background:rgba(255,51,75,0.15); color:#ff334b; border:1px solid #ff334b; border-radius:4px; padding:3px 8px; font-size:0.75rem; cursor:pointer;">🗑️</button>
            </td>
          </tr>`;
        });
        html += '</table>';
        container.innerHTML = html;
      } catch (err) {
        container.innerHTML = '<div style="color:#ff334b; padding:15px;">Failed to load recordings: ' + err.message + '</div>';
      }
    }

    async function deleteMissionRecording(filename) {
      if (!confirm('Are you sure you want to delete ' + filename + '?')) return;
      try {
        await fetch('/api/recordings/' + encodeURIComponent(filename), {method: 'DELETE'});
        loadRecordingsList();
      } catch(err) {
        alert('Delete failed: ' + err.message);
      }
    }