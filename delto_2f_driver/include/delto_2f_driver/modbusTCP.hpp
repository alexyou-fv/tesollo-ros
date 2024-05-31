// #include <boost/asio.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <vector>

class ModbusClient
{
public:

    ModbusClient(const std::string &host, int port)
        : socket_(io_service_), host_(host), port_(port), transaction_id_(0) {}


    virtual ~ModbusClient()
    {
        disconnect();
    }

    bool connect()
    {
        try
        {
            boost::asio::ip::tcp::resolver resolver(io_service_);
            boost::system::error_code ec;

            boost::asio::ip::tcp::resolver::query query(host_, std::to_string(port_));
            boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
            boost::asio::connect(socket_, endpoint_iterator, ec);
            
            std::cout << "Connected to " << host_ << ":" << port_ << std::endl;

            return true;
        }
        catch (std::exception &e)
        {
            std::cerr << "Connection failed: " << e.what() << std::endl;
            return false;
        }
    }

    void disconnect()
    {
        if (socket_.is_open())
        {
            socket_.close();
        }
    }

    std::vector<int16_t> readHoldingRegisters(uint16_t startAddress, uint16_t quantity)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        std::vector<uint8_t> request =
            {
                static_cast<uint8_t>(transaction_id_ >> 8), static_cast<uint8_t>(transaction_id_ & 0xFF), // 트랜잭션 ID
                0x00, 0x00,                                                                               // 프로토콜 ID
                0x00, 0x06,                                                                               // 길이
                0x01,                                                                                     // 단위 ID
                0x03,                                                                                     // 함수 코드 (Read Holding Registers)
                static_cast<uint8_t>(startAddress >> 8), static_cast<uint8_t>(startAddress & 0xFF),
                static_cast<uint8_t>(quantity >> 8), static_cast<uint8_t>(quantity & 0xFF)};

        transaction_id_++;

        boost::asio::write(socket_, boost::asio::buffer(request));

        std::vector<uint8_t> response(5 + quantity * 2);
        size_t reply_length = boost::asio::read(socket_, boost::asio::buffer(response));

        std::vector<int16_t> registers;

        for (size_t i = 0; i < quantity; ++i)
        {
            int16_t value = static_cast<int16_t>(response[3 + i * 2] << 8 | response[4 + i * 2]);
            registers.push_back(value);
        }

        return registers;
    }

    std::vector<int16_t> readInputRegisters(uint16_t startAddress, uint16_t quantity)
    {
        
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<uint8_t> request =
            {
                static_cast<uint8_t>(transaction_id_ >> 8), static_cast<uint8_t>(transaction_id_ & 0xFF), // 트랜잭션 ID
                0x00, 0x00,                                                                               // 프로토콜 ID
                0x00, 0x06,                                                                               // 길이
                0x01,                                                                                     // 단위 ID
                0x04,                                                                                     // 함수 코드 (Read Holding Registers)
                static_cast<uint8_t>(startAddress >> 8), static_cast<uint8_t>(startAddress & 0xFF),
                static_cast<uint8_t>(quantity >> 8), static_cast<uint8_t>(quantity & 0xFF)};

        transaction_id_++;
        //request array


        boost::asio::write(socket_, boost::asio::buffer(request));

        std::vector<uint8_t> response(9 + quantity * 2);
         size_t reply_length = boost::asio::read(socket_, boost::asio::buffer(response));

         
        if((transaction_id_-1) != ((response[0] << 8 )+ response[1]))
            {
             std::vector<int16_t> data;
             data.push_back(0);
             std::cout << "wrong data "<< std::endl;;
             std::cout << "transaction_id_ : " << transaction_id_-1 << std::endl;
                std::cout << ((response[0] << 8 )+ response[1]) << std::endl;
            return data;
        }

       

        std::vector<int16_t> registers;

        //byte array
        auto byteArray = std::vector<uint8_t>(response.begin(), response.end());
        for (size_t i = 0; i < quantity; ++i)
        {
            int16_t value = static_cast<int16_t>(response[9 + i * 2] << 8 | response[10 + i * 2]);
            registers.push_back(value);
        }

        // std::cout <<"test \n" ;
        // printByteArray
        for (size_t i = 0; i < byteArray.size(); ++i)
        {
            std::cout << "byteArray[" << i << "] : " << int(byteArray[i]) << " "<< std::endl;;
        }
        // std::cout << std::endl;

        return registers;   
    }

    
    void writeSingleCoil(uint16_t address, bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        std::vector<uint8_t> request = {
            static_cast<uint8_t>(transaction_id_ >> 8), static_cast<uint8_t>(transaction_id_ & 0xFF), // 트랜잭션 ID
            0x00, 0x00,                                                                               // 프로토콜 ID
            0x00, 0x06,                                                                               // 길이
            0x01,                                                                                     // 단위 ID
            0x05,                                                                                     // 함수 코드 (Write Single Coil)
            static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address & 0xFF),                 // 시작 주소
            static_cast<uint8_t>(value ? 0xFF : 0x00), 0x00                                           // 값 (ON: 0xFF00, OFF: 0x0000)
        };

        transaction_id_++;

        boost::asio::write(socket_, boost::asio::buffer(request));
        // 응답 처리 로직이 필요하면 여기에 추가
    }

    void writeMultiCoils(uint16_t address, const std::vector<bool> &values)
    {
        
        std::lock_guard<std::mutex> lock(mutex_);
        size_t num_bytes = (values.size() + 7) / 8;        // 코일 상태를 저장하기 위한 바이트 수 계산
        std::vector<uint8_t> value_bytes(num_bytes, 0x00); // 코일 상태를 저장할 바이트 배열

        // 코일 상태 설정
        for (size_t i = 0; i < values.size(); i++)
        {
            if (values[i])
            {
                value_bytes[i / 8] |= 1 << (i % 8);
            }
        }

        std::vector<uint8_t> request = 
        {
            static_cast<uint8_t>(transaction_id_ >> 8), static_cast<uint8_t>(transaction_id_ & 0xFF), // 트랜잭션 ID
            0x00, 0x00,                                                                               // 프로토콜 ID
            static_cast<uint8_t>((7 + num_bytes) >> 8), static_cast<uint8_t>(7 + num_bytes & 0xFF),   // 길이
            0x01,                                                                                     // 단위 ID
            0x0F,                                                                                     // 함수 코드 (Write Multiple Coils)
            static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address & 0xFF),                 // 시작 주소
            static_cast<uint8_t>(values.size() >> 8), static_cast<uint8_t>(values.size() & 0xFF),     // 코일 수
            static_cast<uint8_t>(num_bytes),                                                          // 값 바이트 수
        };

        request.insert(request.end(), value_bytes.begin(), value_bytes.end()); // 코일 상태 바이트 추가

        transaction_id_++;

        boost::asio::write(socket_, boost::asio::buffer(request));
        
        // 응답 처리 로직이 필요하면 여기에 추가
    }

    void writeSingleRegister(uint16_t address, uint16_t value)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        std::vector<uint8_t> request =
            {
                static_cast<uint8_t>(transaction_id_ >> 8), static_cast<uint8_t>(transaction_id_ & 0xFF), // 트랜잭션 ID
                0x00, 0x00,                                                                               // 프로토콜 ID
                0x00, 0x06,                                                                               // 길이
                0x01,                                                                                     // 단위 ID
                0x06,                                                                                     // 함수 코드 (Write Single Register)
                static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address & 0xFF),
                static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};

        transaction_id_++;

        boost::asio::write(socket_, boost::asio::buffer(request));
        // 응답 처리 로직이 필요하면 여기에 추가
    }

    void writeMultiRegisters(uint16_t address, const std::vector<uint16_t> &values)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        size_t values_bytes = values.size() * 2; // 각 값은 2바이트

        std::vector<uint8_t> request = {
            static_cast<uint8_t>(transaction_id_ >> 8), static_cast<uint8_t>(transaction_id_ & 0xFF),     // 트랜잭션 ID
            0x00, 0x00,                                                                                   // 프로토콜 ID
            static_cast<uint8_t>((7 + values_bytes) >> 8), static_cast<uint8_t>(7 + values_bytes & 0xFF), // 길이 (헤더 이후의 길이)
            0x01,                                                                                         // 단위 ID
            0x10,                                                                                         // 함수 코드 (Write Multiple Registers)
            static_cast<uint8_t>(address >> 8), static_cast<uint8_t>(address & 0xFF),                     // 시작 주소
            static_cast<uint8_t>(values.size() >> 8), static_cast<uint8_t>(values.size() & 0xFF),         // 레지스터 수
            static_cast<uint8_t>(values_bytes)                                                            // 값의 바이트 수
        };

        // 값을 바이트 배열로 변환하여 요청에 추가
        for (const auto &value : values)
        {
            request.push_back(static_cast<uint8_t>(value >> 8));
            request.push_back(static_cast<uint8_t>(value & 0xFF));
        }

        transaction_id_++; // 트랜잭션 ID 업데이트

        boost::asio::write(socket_, boost::asio::buffer(request)); // 요청 전송

        // TODO: 응답 처리 로직
    }

private:
    boost::asio::io_service io_service_;
    boost::asio::ip::tcp::socket socket_;
    std::string host_;
    uint16_t port_;
    uint16_t transaction_id_;

    std::mutex mutex_;
};


// bool writeMultiRegisters(uint16_t address, const std::vector<uint16_t> &values)
// {
//     // (여기에 기존의 요청 전송 코드를 삽입...)

//     try {
//         // 응답 버퍼 준비
//         std::vector<uint8_t> response(8); // Write Multiple Registers 응답의 기대 크기는 8바이트입니다.
//         size_t reply_length = boost::asio::read(socket_, boost::asio::buffer(response));

//         // 응답 길이 확인
//         if (reply_length != response.size()) {
//             std::cerr << "Invalid response length." << std::endl;
//             return false;
//         }

//         // 응답에서 트랜잭션 ID, 프로토콜 ID, 길이, 단위 ID, 함수 코드 확인
//         if (response[0] == static_cast<uint8_t>(transaction_id_ >> 8) &&
//             response[1] == static_cast<uint8_t>(transaction_id_ & 0xFF) &&
//             response[2] == 0x00 && response[3] == 0x00 &&
//             response[5] == 0x01 && response[6] == 0x10) {
//             // 응답이 성공적인 경우
//             return true;
//         } else {
//             // 에러 코드 처리 (Modbus 예외 응답은 함수 코드의 최상위 비트가 1로 설정됩니다)
//             if ((response[6] & 0x80) != 0) {
//                 uint8_t errorCode = response[7];
//                 std::cerr << "Modbus exception code: " << static_cast<int>(errorCode) << std::endl;
//             } else {
//                 std::cerr << "Mismatch in response." << std::endl;
//             }
//             return false;
//         }
//     } catch (const std::exception& e) {
//         std::cerr << "Exception in response handling: " << e.what() << std::endl;
//         return false;
//     }
// }
