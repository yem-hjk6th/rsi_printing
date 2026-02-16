from datetime import datetime, timedelta


class DataLogger:
    """Handles saving width measurements."""
    @staticmethod
    def save(all_widths, width_mm, timestamp, relative_time, acc_length, fps_constant, length):
        width_val = round(width_mm,2) if width_mm is not None else None
        all_widths.append([int(timestamp), relative_time, width_val, round(acc_length,2)])
        relative_time += fps_constant / 30
        acc_length += length
        dt = datetime.strptime(timestamp, "%Y%m%d%H%M%S%f")
        timestamp = (dt + timedelta(seconds=fps_constant/30)).strftime("%Y%m%d%H%M%S%f")[:-3]
        return all_widths, relative_time, acc_length, timestamp
