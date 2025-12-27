import streamlit as st
import pandas as pd
import plotly.express as px
from pathlib import Path

# --- PAGE CONFIGURATION ---
st.set_page_config(
    page_title="Autonomous UAV IFF System",
    page_icon="🛸",
    layout="wide"
)

# Custom CSS for a "Military-Tech" Aesthetic
st.markdown("""
<style>
    .main { background-color: #0B0E14; color: #E0E0E0; }
    .stTabs [data-baseweb="tab-list"] { gap: 24px; }
    .stTabs [data-baseweb="tab"] {
        height: 50px;
        white-space: pre-wrap;
        background-color: #1A1C23;
        border-radius: 4px 4px 0px 0px;
        color: #8A8D91;
    }
    .stTabs [aria-selected="true"] { background-color: #3B82F6 !important; color: white !important; }
    .metric-card {
        background: linear-gradient(135deg, #1e293b 0%, #0f172a 100%);
        padding: 20px;
        border-radius: 12px;
        border-left: 5px solid #3B82F6;
        box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.5);
        text-align: center;
    }
    .patent-badge {
        background-color: #059669;
        color: white;
        padding: 4px 12px;
        border-radius: 20px;
        font-weight: bold;
        font-size: 0.9em;
    }
</style>
""", unsafe_allow_html=True)

# --- SIDEBAR ---
with st.sidebar:
    st.image("assets/showcase_1.png", caption="Experimental Prototype")
    st.title("Project Summary")
    st.markdown('<span class="patent-badge">📜 2 Patents Published</span>', unsafe_allow_html=True)
    st.info("""
    **Objective:** Solving the 'Authentication Deadlock' in decentralized drone networks using the patented BYCAST-IFF protocol.
    """)
    st.success("**Lead Developer:** Chaitanya Jayant")
    st.caption("TIET | Capstone Project")

# --- HEADER ---
st.title("🛸 Autonomous Friend or Foe (IFF) System")
st.markdown("### Secure Decentralized Identification for UAV Swarms")
st.write("---")

# --- KPI METRICS (REPLACED CARDS) ---
m1, m2, m3, m4 = st.columns(4)
with m1:
    st.markdown('<div class="metric-card"><h4>Auth Success</h4><h2>91.3%</h2><p>Tactical Environment</p></div>', unsafe_allow_html=True)
with m2:
    st.markdown('<div class="metric-card"><h4>Payload Accuracy</h4><h2>< 5m</h2><p>GPS-Guided Drop</p></div>', unsafe_allow_html=True)
with m3:
    # REPLACED: Tech metric for Latency
    st.markdown('<div class="metric-card"><h4>Auth Latency</h4><h2>847ms</h2><p>Avg Handshake Time</p></div>', unsafe_allow_html=True)
with m4:
    # REPLACED: Tech metric for RF Reliability
    st.markdown('<div class="metric-card"><h4>RF Link Success</h4><h2>92.2%</h2><p>LoRa 433MHz Mesh</p></div>', unsafe_allow_html=True)

# --- MAIN TABS ---
st.markdown("<br>", unsafe_allow_html=True)
tab1, tab2, tab3, tab4, tab5 = st.tabs([
    "🏗️ System Architecture", 
    "🛡️ Patented IFF Logic", 
    "📈 Performance Results", 
    "⚙️ Hardware & PCB", 
    "📜 Patents & Docs"
])

with tab1:
    st.header("Unified System Architecture")
    col1, col2 = st.columns([2, 1])
    with col1:
        st.image("assets/full_setup.png", caption="Fig 1.1: Multi-Tier Integrated Architecture")
    with col2:
        st.write("""
        **Two-Tier Edge Architecture:**
        1. **Tier 1 (ESP32):** Real-time flight stabilization, PID control, and AES-256 encryption.
        2. **Tier 2 (Raspberry Pi 4B):** High-level mission planning and GPS-guided payload logistics.
        """)
        st.image("assets/Use_Case_Diagram.png", caption="System Interaction Model")
    
    st.subheader("Process Logic Flow")
    c1, c2 = st.columns(2)
    with c1:
        st.image("assets/Activity_Diagram.png", caption="Flight Controller Behavioral Flow")
    with c2:
        st.image("assets/Data_Flow_Diagram.png", caption="Level 1 DFD: Sensor-to-Motor Signal Chain")

with tab2:
    st.header("The BYCAST-IFF Protocol")
    st.write("Our **Bidirectional Yet Collision-Avoiding Secure Transmission** protocol ensures drones can identify each other without freezing the network.")
    
    col1, col2 = st.columns(2)
    with col1:
        st.subheader("1. Three-Way Handshake")
        st.image("assets/Sequence_Diagram.png", caption="Encrypted Authentication Sequence")
    with col2:
        st.subheader("2. Role-Alternating Mechanism")
        st.image("assets/role_alternation.png", caption="Deadlock Prevention Timeline (15s Interval)")

    st.markdown("---")
    st.subheader("Deterministic RF Collision Avoidance")
    st.write("Using our patented prime-number slotting algorithm, drones calculate unique TDMA slots to avoid RF contention.")
    # Interactive visual of Equation 3.1
    ids = list(range(10, 21))
    offsets = [(i * 997) % 3000 for i in ids]
    df_slots = pd.DataFrame({"Drone ID": ids, "Offset (ms)": offsets})
    fig = px.bar(df_slots, x="Drone ID", y="Offset (ms)", title="Prime-Number Based Slot Distribution", color_discrete_sequence=['#3B82F6'])
    st.plotly_chart(fig, use_container_width=True)

with tab3:
    st.header("Experimental Validation")
    st.write("Testing conducted in indoor (50m) and outdoor (500m) tactical environments.")
    
    col1, col2 = st.columns(2)
    with col1:
        st.image("assets/latency.png", caption="Authentication Latency Distribution (ms)")
    with col2:
        st.image("assets/HardwareInLoop_Simulation_Setup.png", caption="Hardware-in-the-Loop (HIL) Testbed")
    
    st.markdown("---")
    st.subheader("Interface Design")
    c1, c2 = st.columns(2)
    with c1:
        st.image("assets/iff_dashboard.png", caption="IFF Security Monitor Dashboard")
    with c2:
        st.image("assets/mission_dashboard.png", caption="Tactical Mission Planner Dashboard")

with tab4:
    st.header("Custom Hardware Implementation")
    st.write("Designed and fabricated indigenous PCBs to reduce system weight and latency.")
    
    col1, col2 = st.columns(2)
    with col1:
        st.image("assets/fc_schematic.png", caption="ESP32 Flight Controller Schematic")
    with col2:
        st.image("assets/iff_schematic.png", caption="LoRa IFF Module Schematic")
    
    st.markdown("---")
    st.subheader("Final Assembly")
    st.image("assets/drone_2.png", caption="Fully Integrated Hexacopter System", use_container_width=True)

with tab5:
    st.header("Intellectual Property & Documentation")
    st.write("This project resulted in two published patents (Issue No. 29/2025).")
    
    col1, col2 = st.columns(2)
    with col1:
        st.image("assets/fc_patent.png", caption="Patent: Flight Controller System")
    with col2:
        st.image("assets/iff_patent.png", caption="Patent: Secure Drone Identification System")
    
    st.markdown("---")
    st.subheader("Data Modeling")
    st.image("assets/Entity_Diagrama.png", caption="Mission Planner Entity-Relationship Model")

# --- FOOTER ---
st.markdown("---")
st.markdown("""
<div style="text-align: center; color: #64748B; padding: 20px;">
    <p><b>Proprietary Warning:</b> Core source code (C++/Python) is protected under IPR. Contact the lead developer for partnership or licensing.</p>
    <p>© 2025 Autonomous UAV System Team | Thapar Institute of Engineering & Technology</p>
</div>
""", unsafe_allow_html=True)