import sqlite3
import sys
import argparse  # 添加参数解析库

def analyze_database(db_path):
    try:
        # 连接数据库
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        # 查询simINFO表获取基本仿真信息
        print("=== simINFO 表内容 ===")
        cursor.execute("SELECT * FROM simINFO;")
        sim_info = cursor.fetchall()
        if sim_info:
            # 获取列名
            columns = [desc[0] for desc in cursor.description]
            print(columns)
            for row in sim_info:
                print(row)
        else:
            print("simINFO表为空")
        
        # 查询frameINFO表的最后几帧数据，检查是否有异常
        print("\n=== frameINFO 表最后10帧数据 ===")
        cursor.execute("SELECT * FROM frameINFO ORDER BY frame DESC LIMIT 10;")
        frame_info = cursor.fetchall()
        if frame_info:
            columns = [desc[0] for desc in cursor.description]
            print(columns)
            for row in frame_info:
                print(row)
        else:
            print("frameINFO表为空")
        
        # 查询vehicleINFO表检查车辆参数
        print("\n=== vehicleINFO 表内容 ===")
        cursor.execute("SELECT * FROM vehicleINFO;")
        vehicle_info = cursor.fetchall()
        if vehicle_info:
            columns = [desc[0] for desc in cursor.description]
            print(columns)
            for row in vehicle_info:
                print(row)
        else:
            print("vehicleINFO表为空")
        
        conn.close()
    except Exception as e:
        print(f"数据库分析出错: {str(e)}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='分析LimSim仿真数据库文件')
    parser.add_argument('db_path', help='SQLite数据库文件路径')
    args = parser.parse_args()
    analyze_database(args.db_path)
    if len(sys.argv) != 2:
        print("用法: python analyze_db.py <数据库路径>", file=sys.stderr)
        sys.exit(1)
    db_path = sys.argv[1]
    analyze_database(db_path)