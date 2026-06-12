import pandas as pd

file_names = ['thong_so.csv', 'thong_so2.csv', 'thong_so3.csv']

for file in file_names:
    try:
        df = pd.read_csv(file)
        
        # Tính khoảng thời gian giữa các dòng liên tiếp (diff)
        # dropna bỏ qua dòng đầu tiên 
        time_diffs = df['Time_ms'].diff().dropna()
        
        # Thống kê xem có những khoảng thời gian nào
        diff_counts = time_diffs.value_counts().to_dict()
        print("1. Bảng thống kê các khoảng thời gian (T_s):")
        for diff_val, count in diff_counts.items():
            print(f"   -> Khoảng {diff_val} ms: xuất hiện {count} lần")
        
        # Tìm các dòng KHÔNG CÁCH NHAU ĐÚNG 10ms
        anomalies = df[df['Time_ms'].diff() != 10.0].dropna(subset=['Time_ms'])
        
        # Xóa dòng đầu tiên (vì diff của nó luôn là NaN) để tránh báo lỗi giả
        if not anomalies.empty and df['Time_ms'].diff().iloc[0] != 10.0:
             anomalies = anomalies.iloc[1:]
             
        if anomalies.empty:
            print("\n✅ KẾT LUẬN: TUYỆT VỜI! 100% dữ liệu cách nhau đúng 10ms.")
        else:
            print(f"\n⚠️ CẢNH BÁO: Phát hiện {len(anomalies)} dòng bị lệch nhịp 10ms!")
            print("Các dòng bị lỗi (để bạn dễ tra Excel):")
            # In ra Index (số dòng) và Thời gian lúc bị lệch
            print(anomalies[['Time_ms']])
            
    except Exception as e:
        print(f"❌ Lỗi khi đọc file {file}: {e}")