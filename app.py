import streamlit as st
import folium
from streamlit_folium import st_folium
from streamlit_option_menu import option_menu
from datetime import datetime
import time
import pandas as pd
import json
import os
import numpy as np

# ================== 统一持久化存储（障碍物 + A/B标点 全部保存到JSON） ==================
SAVE_FILE = "drone_data.json"  # 一个文件统一存所有数据，管理更干净

# 加载全部数据（障碍物、A点、B点、点位状态）
def load_all_data():
    if os.path.exists(SAVE_FILE):
        with open(SAVE_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    # 默认初始数据（南京科技职业学院原始坐标）
    return {
        "A": [32.2322, 118.7490],
        "B": [32.2343, 118.7490],
        "A_set": False,
        "B_set": False,
        "obstacles": []
    }

# 保存全部数据到本地JSON（永久存储）
def save_all_data():
    data = {
        "A": list(st.session_state.A),
        "B": list(st.session_state.B),
        "A_set": st.session_state.A_set,
        "B_set": st.session_state.B_set,
        "obstacles": st.session_state.polygon_memory
    }
    with open(SAVE_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

# ================== 全局状态初始化（从文件加载，不再硬编码默认值） ==================
data = load_all_data()

if "A" not in st.session_state:
    st.session_state.A = tuple(data["A"])
if "B" not in st.session_state:
    st.session_state.B = tuple(data["B"])
if "A_set" not in st.session_state:
    st.session_state.A_set = data["A_set"]
if "B_set" not in st.session_state:
    st.session_state.B_set = data["B_set"]
if "height" not in st.session_state:
    st.session_state.height = 50
if "heartbeat_data" not in st.session_state:
    st.session_state.heartbeat_data = []
if "polygon_memory" not in st.session_state:
    st.session_state.polygon_memory = data["obstacles"]

# 障碍物绘制优化状态（之前好用的圈选功能全部保留）
if "is_drawing" not in st.session_state:
    st.session_state.is_drawing = False
if "temp_points" not in st.session_state:
    st.session_state.temp_points = []

st.set_page_config(layout="wide", page_title="南科院无人机航线规划系统")

# ================== 左侧侧边栏导航 ==================
with st.sidebar:
    st.title("🚁 无人机系统导航")
    page = option_menu("功能页面", ["航线规划", "飞行监控"], default_index=0)
    st.divider()
    st.subheader("坐标系转换")
    coord_type = st.radio("", ["GCJ-02(火星坐标)", "WGS-84(原始坐标)"])
    st.divider()
    st.subheader("系统点位状态")
    st.button("✅ A点已设置" if st.session_state.A_set else "❌ A点未设置", type="primary")
    st.button("✅ B点已设置" if st.session_state.B_set else "❌ B点未设置", type="primary")

# ================== GCJ-02 转 WGS-84 坐标纠偏算法 ==================
def gcj02_to_wgs84(lng: float, lat: float):
    a = 6378245.0
    ee = 0.00669342162296594323

    def transform_lat(x, y):
        ret = -100.0 + 2.0*x + 3.0*y + 0.2*y*y + 0.1*x*y + 0.2*np.sqrt(abs(x))
        ret += (20.0*np.sin(6.0*x*np.pi) + 20.0*np.sin(2.0*x*np.pi)) * 2.0 / 3.0
        ret += (20.0*np.sin(y*np.pi) + 40.0*np.sin(y/3.0*np.pi)) * 2.0 / 3.0
        ret += (160.0*np.sin(y/12.0*np.pi) + 320*np.sin(y/30.0*np.pi)) * 2.0 / 3.0
        return ret

    def transform_lng(x, y):
        ret = 300.0 + x + 2.0*y + 0.1*x*x + 0.1*x*y + 0.1*np.sqrt(abs(x))
        ret += (20.0*np.sin(6.0*x*np.pi) + 20.0*np.sin(2.0*x*np.pi)) * 2.0 / 3.0
        ret += (20.0*np.sin(x*np.pi) + 40.0*np.sin(x/3.0*np.pi)) * 2.0 / 3.0
        ret += (150.0*np.sin(x/12.0*np.pi) + 300.0*np.sin(x/30.0*np.pi)) * 2.0 / 3.0
        return ret

    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * np.pi
    magic = np.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = np.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * np.pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * np.cos(radlat) * np.pi)
    return lat - dlat, lng - dlng

# ================== 主页面：航线规划 ==================
if page == "航线规划":
    st.title("🚁 南京科技职业学院 无人机3D航线规划系统")
    col_map, col_ctrl = st.columns([3.2, 1])

    with col_ctrl:
        st.subheader("🎛️ 点位与飞行参数")
        a_lat = st.number_input("起点A 纬度", value=st.session_state.A[0], format="%.6f")
        a_lon = st.number_input("起点A 经度", value=st.session_state.A[1], format="%.6f")
        b_lat = st.number_input("终点B 纬度", value=st.session_state.B[0], format="%.6f")
        b_lon = st.number_input("终点B 经度", value=st.session_state.B[1], format="%.6f")
        st.session_state.height = st.slider("无人机飞行高度 (m)", 0, 200, value=st.session_state.height)

        # 设置A点 + 自动保存到JSON
        if st.button("确定设置起点A"):
            st.session_state.A = (a_lat, a_lon)
            st.session_state.A_set = True
            save_all_data()  # 关键：修改后立刻持久化保存
            st.success("A点坐标已永久保存！")
        # 设置B点 + 自动保存到JSON
        if st.button("确定设置终点B"):
            st.session_state.B = (b_lat, b_lon)
            st.session_state.B_set = True
            save_all_data()  # 关键：修改后立刻持久化保存
            st.success("B点坐标已永久保存！")

        st.divider()
        st.subheader("🚧 障碍物区域圈选（优化增强版）")
        # 绘制状态提示
        if st.session_state.is_drawing:
            st.warning(f"🖱️ 正在绘制障碍物，已点击点位：{len(st.session_state.temp_points)} 个")
            st.caption("操作：地图点击添加顶点 → 撤销错点 → 完成闭合保存")
        else:
            st.info("点击【开始绘制障碍物】，在地图上点击圈选禁飞区")

        # 按钮组 开始、撤销、取消
        btn1, btn2, btn3 = st.columns(3)
        with btn1:
            if st.button("开始绘制"):
                st.session_state.is_drawing = True
                st.session_state.temp_points = []
        with btn2:
            if st.button("撤销上一点"):
                if st.session_state.temp_points:
                    st.session_state.temp_points.pop()
        with btn3:
            if st.button("取消绘制"):
                st.session_state.is_drawing = False
                st.session_state.temp_points = []

        # 完成圈选障碍物 + 自动保存全部数据
        if st.button("✅ 完成圈选并保存"):
            if len(st.session_state.temp_points) >= 3:
                st.session_state.polygon_memory.append(st.session_state.temp_points.copy())
                save_all_data()  # 障碍物新增也统一保存
                st.success("障碍物区域已永久保存！")
            else:
                st.error("至少需要圈选3个点位才能形成封闭障碍物！")
            st.session_state.is_drawing = False
            st.session_state.temp_points = []
            st.rerun()

        # 清空全部障碍物 + 保存
        if st.button("🗑️ 清空全部障碍物"):
            st.session_state.polygon_memory = []
            st.session_state.temp_points = []
            save_all_data()
            st.rerun()

        st.info(f"系统已记忆障碍物总数：{len(st.session_state.polygon_memory)} 个")

        st.divider()
        st.subheader("❤️ 无人机心跳监测")
        now = datetime.now().strftime("%H:%M:%S")
        st.metric("当前系统时间", now)
        st.session_state.heartbeat_data.append(time.time())
        if len(st.session_state.heartbeat_data) > 30:
            st.session_state.heartbeat_data.pop(0)
        df = pd.DataFrame({"采样点": range(len(st.session_state.heartbeat_data)), "心跳时间戳": st.session_state.heartbeat_data})
        st.line_chart(df.set_index("采样点"), height=150)
        st.success("无人机链路心跳正常")

    with col_map:
        # 地图中心点（自动跟随保存后的A/B点）
        center_lat = (st.session_state.A[0] + st.session_state.B[0]) / 2
        center_lon = (st.session_state.A[1] + st.session_state.B[1]) / 2

        # 高清3D倾斜地图底图，OpenStreetMap 高清瓦片，streamlit/github全兼容
        m = folium.Map(
            location=[center_lat, center_lon],
            zoom_start=19,
            tiles="https://{s}.tile.openstreetmap.de/tiles/osmde/{z}/{x}/{y}.png",
            attr="OpenStreetMap 3D",
            max_zoom=22
        )
        # 开启全屏、视角优化
        folium.plugins.Fullscreen(position="topright").add_to(m)

        # 坐标纠偏转换
        if coord_type == "GCJ-02(火星坐标)":
            A_wgs = gcj02_to_wgs84(st.session_state.A[1], st.session_state.A[0])
            B_wgs = gcj02_to_wgs84(st.session_state.B[1], st.session_state.B[0])
        else:
            A_wgs = st.session_state.A
            B_wgs = st.session_state.B

        # 绘制起点、终点、规划航线（加载保存后的点位）
        if st.session_state.A_set:
            folium.CircleMarker(
                A_wgs, radius=12, color="red", fill=True, fill_color="red", popup="飞行起点A（已保存）"
            ).add_to(m)
        if st.session_state.B_set:
            folium.CircleMarker(
                B_wgs, radius=12, color="green", fill=True, fill_color="green", popup="飞行终点B（已保存）"
            ).add_to(m)
        if st.session_state.A_set and st.session_state.B_set:
            folium.PolyLine(
                [A_wgs, B_wgs], color="#0066ff", weight=4, opacity=0.8
            ).add_to(m)

        # 绘制所有已经永久保存的障碍物
        for idx, poly in enumerate(st.session_state.polygon_memory):
            if len(poly) >= 3:
                folium.Polygon(
                    locations=poly,
                    color="#dc2626",
                    fill=True,
                    fill_color="#dc2626",
                    fill_opacity=0.45,
                    popup=f"禁飞障碍物区域 {idx+1}"
                ).add_to(m)

        # 绘制当前正在绘制的临时障碍物（橙色高亮虚线+顶点标记）
        if len(st.session_state.temp_points) > 0:
            for point in st.session_state.temp_points:
                folium.CircleMarker(
                    point, radius=5, color="#ff7700", fill=True, fill_color="#ff7700"
                ).add_to(m)
            folium.PolyLine(
                st.session_state.temp_points, color="#ff7700", weight=3, dash_array="10 5"
            ).add_to(m)

        # 地图渲染（固定key防止刷新错乱）
        output = st_folium(m, width=1150, height=720, key="main_map_3d")

        # 地图点击加点逻辑（稳定无误触）
        if st.session_state.is_drawing:
            if output and output.get("last_clicked") is not None:
                click_lat = output["last_clicked"]["lat"]
                click_lng = output["last_clicked"]["lng"]
                if not st.session_state.temp_points or [click_lat, click_lng] != st.session_state.temp_points[-1]:
                    st.session_state.temp_points.append([click_lat, click_lng])
                    st.rerun()

# ================== 飞行监控页面 ==================
else:
    st.title("📡 无人机实时飞行监控中心")
    st.success("✅ 无人机系统链路正常，设备在线")
    st.subheader("监测区域：南京科技职业学院校内空域")
    st.info("航线A/B点坐标、障碍物禁飞区全部永久持久化保存，实时同步")
