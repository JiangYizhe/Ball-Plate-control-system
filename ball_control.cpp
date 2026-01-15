/**
 * 板球控制系统 - 修复版（舵机方向修正）
 * 修复问题：
 * 1. 舵机转向反向 - 反转控制输出
 * 2. 程序无法正常停止 - 添加了正确的信号处理和退出机制
 * 3. 坐标解析错误 - 修复了小端序解析逻辑
 * 4. 串口数据读取稳定性 - 改进了数据包解析
 * 编译：g++ -std=c++14 -pthread -o ball_control_fixed ball_control_fixed.cpp
 * 运行：sudo ./ball_control_fixed
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <queue>
#include <memory>
#include <cstring>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <csignal>
#include <signal.h>
#include <cstdlib>
#include <iomanip>

using namespace std;
using namespace chrono;

// ====================== 全局运行标志 ======================
atomic<bool> g_running{true};
atomic<bool> g_stop_requested{false};

// ====================== 信号处理函数 ======================
void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        cout << "\n收到停止信号，正在关闭系统..." << endl;
        g_running = false;
        g_stop_requested = true;
    }
}

// ====================== 配置参数 ======================
struct Config {
    int debug_level = 1;               // 调试级别
    int image_width = 320;             // QVGA宽度
    int image_height = 240;            // QVGA高度
    int target_x = 185;                // 目标X（修正为185）
    int target_y = 120;                // 目标Y
    float tolerance = 5.0f;            // 容差
    
    float pid_x_kp = 0.5f;             // X轴PID
    float pid_x_ki = 0.01f;
    float pid_x_kd = 0.25f;
    float pid_x_max = 20.0f;
    float pid_x_min = -20.0f;
    
    float pid_y_kp = 0.5f;             // Y轴PID
    float pid_y_ki = 0.01f;
    float pid_y_kd = 0.25f;
    float pid_y_max = 20.0f;
    float pid_y_min = -20.0f;
    
    int servo1_pwm_chip = 3;           // 舵机1
    int servo1_pwm_channel = 0;
    int servo2_pwm_chip = 4;           // 舵机2
    int servo2_pwm_channel = 0;
    
    int servo_center_pulse = 1500000;  // 中位
    int servo_min_pulse = 500000;      // 最小
    int servo_max_pulse = 2500000;     // 最大
    float servo_angle_range = 180.0f;  // 范围
    
    string serial_port = "/dev/ttyACM0"; // 串口
    int serial_baud = 115200;          // 波特率
    
    float control_frequency = 50.0f;   // 频率
    float sample_time = 0.02f;         // 采样时间
    float filter_alpha = 0.2f;         // 滤波
    
    // 新增：舵机方向控制（修复反向问题）
    bool invert_servo_x = true;        // X舵机反向
    bool invert_servo_y = true;        // Y舵机反向
    
    void print() {
        cout << "=== 系统配置 ===" << endl;
        cout << "串口: " << serial_port << " @ " << serial_baud << "bps" << endl;
        cout << "舵机1: pwmchip" << servo1_pwm_chip << "/pwm" << servo1_pwm_channel 
             << (invert_servo_x ? " (反向)" : "") << endl;
        cout << "舵机2: pwmchip" << servo2_pwm_chip << "/pwm" << servo2_pwm_channel 
             << (invert_servo_y ? " (反向)" : "") << endl;
        cout << "目标位置: (" << target_x << ", " << target_y << ")" << endl;
        cout << "控制频率: " << control_frequency << "Hz" << endl;
        cout << "=================" << endl;
    }
};

// ====================== 数据结构 ======================
struct Point2D {
    float x, y;
    Point2D(float _x = 0, float _y = 0) : x(_x), y(_y) {}
    
    float distance(const Point2D& other) const {
        return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
    }
};

struct BallData {
    Point2D position;
    steady_clock::time_point timestamp;
    
    BallData() : position(0, 0), timestamp(steady_clock::now()) {}
    
    void update(const Point2D& new_pos) {
        position = new_pos;
        timestamp = steady_clock::now();
    }
    
    float getAgeSeconds() const {
        auto now = steady_clock::now();
        return duration_cast<milliseconds>(now - timestamp).count() / 1000.0f;
    }
};

// ====================== PID控制器 ======================
class PIDController {
private:
    float kp, ki, kd;
    float max_output, min_output;
    float integral = 0;
    float prev_error = 0;
    float prev_time = 0;
    float sample_time = 0.02f;
    
public:
    PIDController(float p, float i, float d, float max, float min, float dt = 0.02f)
        : kp(p), ki(i), kd(d), max_output(max), min_output(min), sample_time(dt) {}
    
    void setGains(float p, float i, float d) {
        kp = p; ki = i; kd = d;
    }
    
    float calculate(float error, float current_time) {
        float dt = current_time - prev_time;
        if (dt <= 0) dt = sample_time;
        
        integral += error * dt;
        if (integral > max_output) integral = max_output;
        if (integral < min_output) integral = min_output;
        
        float derivative = (error - prev_error) / dt;
        float output = kp * error + ki * integral + kd * derivative;
        
        if (output > max_output) output = max_output;
        if (output < min_output) output = min_output;
        
        prev_error = error;
        prev_time = current_time;
        
        return output;
    }
    
    void reset() {
        integral = 0;
        prev_error = 0;
        prev_time = 0;
    }
};

// ====================== 低通滤波器 ======================
class LowPassFilter {
private:
    float alpha;
    float filtered_value = 0;
    bool initialized = false;
    
public:
    LowPassFilter(float a = 0.1f) : alpha(a) {}
    
    float filter(float new_value) {
        if (!initialized) {
            filtered_value = new_value;
            initialized = true;
        } else {
            filtered_value = alpha * new_value + (1 - alpha) * filtered_value;
        }
        return filtered_value;
    }
    
    void reset() {
        filtered_value = 0;
        initialized = false;
    }
    
    float getValue() const { return filtered_value; }
};

// ====================== PWM舵机控制器 ======================
class ServoController {
private:
    int pwm_chip;
    int pwm_channel;
    string pwm_path;
    float angle_range;
    int center_pulse;
    int min_pulse;
    int max_pulse;
    bool initialized = false;
    
    float pulseToAngle(int pulse) const {
        return ((pulse - center_pulse) * angle_range) / (max_pulse - min_pulse);
    }
    
    int angleToPulse(float angle) const {
        return center_pulse + (angle * (max_pulse - min_pulse)) / angle_range;
    }
    
    bool writePWMFile(const string& filename, const string& value) {
        string filepath = pwm_path + filename;
        ofstream file(filepath);
        if (!file.is_open()) {
            return false;
        }
        file << value;
        file.close();
        return true;
    }
    
public:
    ServoController(int chip, int channel, int center, int min, int max, float range)
        : pwm_chip(chip), pwm_channel(channel), center_pulse(center), 
          min_pulse(min), max_pulse(max), angle_range(range) {
        pwm_path = "/sys/class/pwm/pwmchip" + to_string(chip) + 
                   "/pwm" + to_string(channel) + "/";
    }
    
    bool initialize() {
        if (initialized) return true;
        
        // 导出设备
        string export_path = "/sys/class/pwm/pwmchip" + to_string(pwm_chip) + "/export";
        ofstream export_file(export_path);
        if (!export_file.is_open()) {
            // 可能已导出
            cout << "PWM设备可能已导出: pwmchip" << pwm_chip << "/pwm" << pwm_channel << endl;
        } else {
            export_file << pwm_channel;
            export_file.close();
            this_thread::sleep_for(chrono::milliseconds(50));
        }
        
        // 检查设备是否存在
        ifstream check_file(pwm_path + "period");
        if (!check_file.is_open()) {
            cerr << "PWM设备创建失败: " << pwm_path << endl;
            return false;
        }
        
        // 设置参数
        if (!writePWMFile("period", "20000000")) return false;
        if (!writePWMFile("polarity", "normal")) return false;
        if (!writePWMFile("duty_cycle", to_string(center_pulse))) return false;
        if (!writePWMFile("enable", "1")) return false;
        
        initialized = true;
        cout << "舵机初始化成功: pwmchip" << pwm_chip << "/pwm" << pwm_channel << endl;
        return true;
    }
    
    bool setAngle(float angle) {
        if (!initialized) return false;
        
        angle = max(min(angle, angle_range/2), -angle_range/2);
        int pulse = angleToPulse(angle);
        pulse = max(min(pulse, max_pulse), min_pulse);
        
        return writePWMFile("duty_cycle", to_string(pulse));
    }
    
    void cleanup() {
        if (!initialized) return;
        
        setAngle(0);
        this_thread::sleep_for(chrono::milliseconds(100));
        writePWMFile("enable", "0");
        
        string unexport_path = "/sys/class/pwm/pwmchip" + to_string(pwm_chip) + "/unexport";
        ofstream unexport_file(unexport_path);
        if (unexport_file.is_open()) {
            unexport_file << pwm_channel;
        }
        initialized = false;
    }
    
    ~ServoController() {
        cleanup();
    }
};

// ====================== 串口通信 ======================
class SerialPort {
private:
    int fd = -1;
    string port;
    int baud;
    atomic<bool> running{false};
    thread read_thread;
    queue<Point2D> data_queue;
    mutex queue_mutex;
    vector<uint8_t> buffer;
    
    // OpenMV协议参数
    const uint8_t HEAD = 0xFF;
    const uint8_t TAIL = 0xFE;
    const size_t PACKET_SIZE = 7;  // HEAD(1)+LEN(1)+X(2)+Y(2)+TAIL(1)
    
    void setBaudRate(int baud_rate) {
        int speed = B115200;
        if (baud_rate == 9600) speed = B9600;
        else if (baud_rate == 19200) speed = B19200;
        else if (baud_rate == 38400) speed = B38400;
        else if (baud_rate == 57600) speed = B57600;
        
        struct termios tty;
        tcgetattr(fd, &tty);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        tcsetattr(fd, TCSANOW, &tty);
    }
    
    bool parsePacket(const vector<uint8_t>& packet, Point2D& point) {
        if (packet.size() != PACKET_SIZE) return false;
        if (packet[0] != HEAD || packet[6] != TAIL || packet[1] != 0x04) return false;
        
        // 小端序解析：packet[2]是X低字节，packet[3]是X高字节
        // packet[4]是Y低字节，packet[5]是Y高字节
        uint16_t x = (packet[3] << 8) | packet[2];
        uint16_t y = (packet[5] << 8) | packet[4];
        
        point.x = static_cast<float>(x);
        point.y = static_cast<float>(y);
        
        return true;
    }
    
    void readData() {
        uint8_t read_buffer[256];
        
        while (running) {
            int bytes_available = 0;
            ioctl(fd, FIONREAD, &bytes_available);
            
            if (bytes_available > 0) {
                int n = read(fd, read_buffer, min(255, bytes_available));
                if (n > 0) {
                    // 添加调试输出
                    if (false) {  // 可以开启原始数据调试
                        cout << "收到" << n << "字节: ";
                        for (int i = 0; i < n; i++) {
                            cout << hex << setw(2) << setfill('0') << (int)read_buffer[i] << " ";
                        }
                        cout << dec << endl;
                    }
                    
                    // 处理数据
                    for (int i = 0; i < n; i++) {
                        buffer.push_back(read_buffer[i]);
                        
                        // 检查是否有完整数据包
                        if (buffer.size() >= PACKET_SIZE) {
                            // 查找包头
                            size_t start = 0;
                            for (; start <= buffer.size() - PACKET_SIZE; start++) {
                                if (buffer[start] == HEAD) break;
                            }
                            
                            if (start > 0) {
                                // 清除无效数据
                                buffer.erase(buffer.begin(), buffer.begin() + start);
                                if (buffer.size() < PACKET_SIZE) continue;
                            }
                            
                            // 提取数据包
                            vector<uint8_t> packet(buffer.begin(), buffer.begin() + PACKET_SIZE);
                            Point2D point;
                            
                            if (parsePacket(packet, point)) {
                                lock_guard<mutex> lock(queue_mutex);
                                data_queue.push(point);
                            }
                            
                            // 移除已处理的数据
                            buffer.erase(buffer.begin(), buffer.begin() + PACKET_SIZE);
                        }
                    }
                }
            }
            
            this_thread::sleep_for(chrono::milliseconds(1));
        }
    }
    
public:
    SerialPort(const string& p, int b) : port(p), baud(b) {}
    
    bool open() {
        fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd < 0) {
            cerr << "无法打开串口: " << port << " (错误: " << strerror(errno) << ")" << endl;
            return false;
        }
        
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        
        if (tcgetattr(fd, &tty) != 0) {
            cerr << "获取串口属性失败" << endl;
            close();
            return false;
        }
        
        setBaudRate(baud);
        
        tty.c_cflag &= ~PARENB;  // 无校验
        tty.c_cflag &= ~CSTOPB;  // 1停止位
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;      // 8数据位
        tty.c_cflag &= ~CRTSCTS; // 无硬件流控
        tty.c_cflag |= CREAD | CLOCAL;
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
        
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;  // 0.1秒超时
        
        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            cerr << "设置串口属性失败" << endl;
            close();
            return false;
        }
        
        // 清空缓冲区
        tcflush(fd, TCIOFLUSH);
        
        running = true;
        read_thread = thread(&SerialPort::readData, this);
        
        cout << "串口打开成功: " << port << " @ " << baud << "bps" << endl;
        return true;
    }
    
    bool hasData() {
        lock_guard<mutex> lock(queue_mutex);
        return !data_queue.empty();
    }
    
    Point2D readPoint() {
        lock_guard<mutex> lock(queue_mutex);
        if (data_queue.empty()) return Point2D(0, 0);
        
        Point2D point = data_queue.front();
        data_queue.pop();
        return point;
    }
    
    void close() {
        running = false;
        if (read_thread.joinable()) {
            read_thread.join();
        }
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }
    
    ~SerialPort() {
        close();
    }
};

// ====================== 主控制系统 ======================
class BallControlSystem {
private:
    Config config;
    
    unique_ptr<SerialPort> serial;
    unique_ptr<ServoController> servo_x;
    unique_ptr<ServoController> servo_y;
    
    unique_ptr<PIDController> pid_x;
    unique_ptr<PIDController> pid_y;
    unique_ptr<LowPassFilter> filter_x;
    unique_ptr<LowPassFilter> filter_y;
    
    BallData ball_data;
    Point2D target_position;
    atomic<bool> running{false};
    thread control_thread;
    mutex data_mutex;
    
    float servo_x_angle = 0;
    float servo_y_angle = 0;
    int frames_received = 0;
    int frames_processed = 0;
    int invalid_packets = 0;
    float last_control_time = 0;
    
    void controlLoop() {
        auto start_time = steady_clock::now();
        float last_time = 0;
        
        while (running && g_running) {
            auto now = steady_clock::now();
            float current_time = duration_cast<milliseconds>(now - start_time).count() / 1000.0f;
            float dt = current_time - last_time;
            
            if (dt >= config.sample_time) {
                performControl(current_time);
                last_time = current_time;
            }
            
            this_thread::sleep_for(milliseconds(1));
        }
        
        // 停止舵机
        if (servo_x) servo_x->setAngle(0);
        if (servo_y) servo_y->setAngle(0);
    }
    
    void performControl(float current_time) {
        lock_guard<mutex> lock(data_mutex);
        
        // 检查数据新鲜度
        if (ball_data.getAgeSeconds() > 0.5f) {
            // 数据太旧，停止控制
            if (config.debug_level >= 2) {
                cout << "数据过期，暂停控制" << endl;
            }
            return;
        }
        
        float error_x = target_position.x - ball_data.position.x;
        float error_y = target_position.y - ball_data.position.y;
        
        // 滤波
        error_x = filter_x->filter(error_x);
        error_y = filter_y->filter(error_y);
        
        // PID控制
        float control_x = pid_x->calculate(error_x, current_time);
        float control_y = pid_y->calculate(error_y, current_time);
        
        // 关键修复：根据配置反转舵机方向
        if (config.invert_servo_x) control_x = -control_x;
        if (config.invert_servo_y) control_y = -control_y;
        
        // 设置舵机
        servo_x_angle = control_x;
        servo_y_angle = control_y;
        
        servo_x->setAngle(servo_x_angle);
        servo_y->setAngle(servo_y_angle);
        
        last_control_time = current_time;
        frames_processed++;
        
        // 调试输出
        if (config.debug_level >= 2) {
            cout << "控制输出: X=" << control_x << "°, Y=" << control_y << "°" << endl;
        }
    }
    
public:
    BallControlSystem(const Config& cfg) : config(cfg) {
        target_position.x = config.target_x;
        target_position.y = config.target_y;
    }
    
    bool initialize() {
        cout << "正在初始化系统..." << endl;
        
        // 初始化串口
        serial = unique_ptr<SerialPort>(new SerialPort(config.serial_port, config.serial_baud));
        if (!serial->open()) {
            cerr << "错误: 无法打开串口 " << config.serial_port << endl;
            cerr << "尝试检测可用串口..." << endl;
            
            vector<string> test_ports = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"};
            bool found = false;
            
            for (const auto& port : test_ports) {
                serial.reset(new SerialPort(port, config.serial_baud));
                if (serial->open()) {
                    config.serial_port = port;
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                cerr << "未找到可用的串口设备" << endl;
                return false;
            }
        }
        
        // 初始化舵机
        servo_x = unique_ptr<ServoController>(new ServoController(
            config.servo1_pwm_chip, config.servo1_pwm_channel,
            config.servo_center_pulse, config.servo_min_pulse, 
            config.servo_max_pulse, config.servo_angle_range
        ));
        
        servo_y = unique_ptr<ServoController>(new ServoController(
            config.servo2_pwm_chip, config.servo2_pwm_channel,
            config.servo_center_pulse, config.servo_min_pulse,
            config.servo_max_pulse, config.servo_angle_range
        ));
        
        if (!servo_x->initialize()) {
            cerr << "警告: 舵机X初始化失败" << endl;
        }
        if (!servo_y->initialize()) {
            cerr << "警告: 舵机Y初始化失败" << endl;
        }
        
        // 初始化PID
        pid_x = unique_ptr<PIDController>(new PIDController(
            config.pid_x_kp, config.pid_x_ki, config.pid_x_kd,
            config.pid_x_max, config.pid_x_min, config.sample_time
        ));
        
        pid_y = unique_ptr<PIDController>(new PIDController(
            config.pid_y_kp, config.pid_y_ki, config.pid_y_kd,
            config.pid_y_max, config.pid_y_min, config.sample_time
        ));
        
        // 初始化滤波器
        filter_x = unique_ptr<LowPassFilter>(new LowPassFilter(config.filter_alpha));
        filter_y = unique_ptr<LowPassFilter>(new LowPassFilter(config.filter_alpha));
        
        config.print();
        cout << "系统初始化完成" << endl;
        return true;
    }
    
    void start() {
        running = true;
        control_thread = thread(&BallControlSystem::controlLoop, this);
        
        cout << "系统已启动，等待OpenMV数据..." << endl;
        cout << "按Ctrl+C停止系统" << endl;
        
        auto start_time = steady_clock::now();
        auto last_status_time = start_time;
        
        while (running && g_running) {
            // 读取串口数据
            if (serial->hasData()) {
                Point2D new_point = serial->readPoint();
                
                // 验证坐标有效性
                if (new_point.x >= 0 && new_point.x <= config.image_width &&
                    new_point.y >= 0 && new_point.y <= config.image_height) {
                    
                    lock_guard<mutex> lock(data_mutex);
                    ball_data.update(new_point);
                    frames_received++;
                    
                    if (config.debug_level >= 1) {
                        float dist = ball_data.position.distance(target_position);
                        cout << "小球位置: (" << new_point.x << ", " << new_point.y 
                             << "), 距离目标: " << dist << "像素" << endl;
                    }
                    
                    if (ball_data.position.distance(target_position) < config.tolerance) {
                        if (config.debug_level >= 1) {
                            cout << "小球已到达目标位置附近" << endl;
                        }
                    }
                } else {
                    invalid_packets++;
                    if (config.debug_level >= 2) {
                        cout << "无效坐标: (" << new_point.x << ", " << new_point.y << ")" << endl;
                    }
                }
            }
            
            // 状态输出
            auto now = steady_clock::now();
            auto elapsed = duration_cast<seconds>(now - last_status_time).count();
            if (elapsed >= 2) {  // 每2秒输出一次
                printStatus();
                last_status_time = now;
            }
            
            this_thread::sleep_for(milliseconds(1));
        }
        
        stop();
    }
    
    void stop() {
        running = false;
        
        if (control_thread.joinable()) {
            control_thread.join();
        }
        
        // 回中
        if (servo_x) servo_x->setAngle(0);
        if (servo_y) servo_y->setAngle(0);
        
        this_thread::sleep_for(chrono::milliseconds(200));
        
        cout << "系统已停止" << endl;
    }
    
    void printStatus() {
        lock_guard<mutex> lock(data_mutex);
        
        float error_x = target_position.x - ball_data.position.x;
        float error_y = target_position.y - ball_data.position.y;
        float distance = ball_data.position.distance(target_position);
        float data_age = ball_data.getAgeSeconds();
        
        cout << "\n=== 系统状态 ===" << endl;
        cout << "小球位置: (" << fixed << setprecision(1) << ball_data.position.x 
             << ", " << ball_data.position.y << ")" << endl;
        cout << "目标位置: (" << target_position.x << ", " << target_position.y << ")" << endl;
        cout << "位置误差: X=" << error_x << ", Y=" << error_y << ", 距离=" << distance << endl;
        cout << "舵机角度: X=" << servo_x_angle << "°, Y=" << servo_y_angle << "°" << endl;
        cout << "数据年龄: " << data_age << "秒" << endl;
        cout << "接收帧数: " << frames_received << ", 处理帧数: " << frames_processed << endl;
        cout << "无效包数: " << invalid_packets << endl;
        cout << "=================\n" << endl;
    }
    
    ~BallControlSystem() {
        stop();
    }
};

// ====================== 主程序 ======================
int main() {
    // 检查权限
    if (geteuid() != 0) {
        cerr << "请使用sudo权限运行: sudo ./ball_control_fixed" << endl;
        return 1;
    }
    
    // 设置信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    cout << "\n=========================================" << endl;
    cout << "      板球控制系统 (舵机方向修复版)" << endl;
    cout << "      按 Ctrl+C 停止程序" << endl;
    cout << "=========================================\n" << endl;
    
    // 加载配置
    Config config;
    config.debug_level = 1;  // 设置为1以显示基本信息
    config.invert_servo_x = true;  // X舵机反向
    config.invert_servo_y = true;  // Y舵机反向
    
    // 自动检测串口
    vector<string> test_ports = {"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"};
    bool port_found = false;
    
    for (const auto& port : test_ports) {
        if (access(port.c_str(), F_OK) == 0) {
            config.serial_port = port;
            port_found = true;
            cout << "检测到串口设备: " << port << endl;
            break;
        }
    }
    
    if (!port_found) {
        cerr << "未找到串口设备，请连接OpenMV摄像头" << endl;
        return 1;
    }
    
    // 创建控制系统
    BallControlSystem system(config);
    
    try {
        if (system.initialize()) {
            system.start();
        } else {
            cerr << "系统初始化失败" << endl;
            return 1;
        }
    } catch (const exception& e) {
        cerr << "程序异常: " << e.what() << endl;
        return 1;
    }
    
    cout << "程序正常退出" << endl;
    return 0;
}