#include <Arduino.h>

/**
 * The example demonstrates how to port LVGL.
 *
 * ## How to Use
 *
 * To use this example, please firstly install `ESP32_Display_Panel` (including its dependent libraries) and
 * `lvgl` (v8.3.x) libraries, then follow the steps to configure them:
 *
 * 1. [Configure ESP32_Display_Panel](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-esp32_display_panel)
 * 2. [Configure LVGL](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-lvgl)
 * 3. [Configure Board](https://github.com/esp-arduino-libs/ESP32_Display_Panel#configure-board)
 *
 * ## Example Output
 *
 * ```bash
 * ...
 * Hello LVGL! V8.3.8
 * I am ESP32_Display_Panel
 * Starting LVGL task
 * Setup done
 * Loop
 * Loop
 * Loop
 * Loop
 * ...
 * ```
 */

#include <lvgl.h>
#include <ESP_Panel_Library.h>
#include <ESP_IOExpander_Library.h>
#include <ui.h>

#include <painlessMesh.h>
#include <ArduinoJson.h>  // 版本6.19.4

#define   MESH_SSID       "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

// Prototypes
void sendMessage(); 
void receivedCallback(uint32_t from, String & msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback(); 
void nodeTimeAdjustedCallback(int32_t offset); 
void delayReceivedCallback(uint32_t from, int32_t delay);

Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;

bool calc_delay = false;
SimpleList<uint32_t> nodes;
uint32_t RemoteControlNodeID; // 声明全局变量
uint32_t MainBoardID;       // 声明全局变量
String receivedCommand; // 声明全局变量以存储接收到的命令

void sendMessage() ; // Prototype
// Task taskSendMessage( TASK_SECOND * 1, TASK_FOREVER, &sendMessage ); // start with a one second interval
Task taskSendMessage( 10, TASK_FOREVER, &sendMessage ); // start with a one second interval

String command;

// Extend IO Pin define
#define TP_RST 1
#define LCD_BL 2
#define LCD_RST 3
#define SD_CS 4
#define USB_SEL 5

// I2C Pin define
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_SCL_IO 9

/**
/* To use the built-in examples and demos of LVGL uncomment the includes below respectively.
 * You also need to copy `lvgl/examples` to `lvgl/src/examples`. Similarly for the demos `lvgl/demos` to `lvgl/src/demos`.
 */
// #include <demos/lv_demos.h>
// #include <examples/lv_examples.h>

/* LVGL porting configurations */
#define LVGL_TICK_PERIOD_MS     (2)
#define LVGL_TASK_MAX_DELAY_MS  (1000)
#define LVGL_TASK_MIN_DELAY_MS  (1)
#define LVGL_TASK_STACK_SIZE    (4 * 1024)
#define LVGL_TASK_PRIORITY      (2)
#define LVGL_BUF_SIZE           (ESP_PANEL_LCD_H_RES * 20)

ESP_Panel *panel = NULL;
SemaphoreHandle_t lvgl_mux = NULL;                  // LVGL mutex

#if ESP_PANEL_LCD_BUS_TYPE == ESP_PANEL_BUS_TYPE_RGB
/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    lv_disp_flush_ready(disp);
}
#else
/* Display flushing */
void lvgl_port_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    panel->getLcd()->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
}

bool notify_lvgl_flush_ready(void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}
#endif /* ESP_PANEL_LCD_BUS_TYPE */

#if ESP_PANEL_USE_LCD_TOUCH
/* Read the touchpad */
void lvgl_port_tp_read(lv_indev_drv_t * indev, lv_indev_data_t * data)
{
    panel->getLcdTouch()->readData();

    bool touched = panel->getLcdTouch()->getTouchState();
    if(!touched) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        TouchPoint point = panel->getLcdTouch()->getPoint();

        data->state = LV_INDEV_STATE_PR;
        /*Set the coordinates*/
        data->point.x = point.x;
        data->point.y = point.y;

        Serial.printf("Touch point: x %d, y %d\n", point.x, point.y);
    }
}
#endif

void lvgl_port_lock(int timeout_ms)
{
    const TickType_t timeout_ticks = (timeout_ms < 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks);
}

void lvgl_port_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

void lvgl_port_task(void *arg)
{
    Serial.println("Starting LVGL task");

    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        lvgl_port_lock(-1);
        task_delay_ms = lv_timer_handler();
        // Release the mutex
        lvgl_port_unlock();
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void setup()
{
    Serial.begin(115200); /* prepare for possible serial debug */

    mesh.setDebugMsgTypes(ERROR | DEBUG);  // set before init() so that you can see error messages

    mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
    mesh.onNodeDelayReceived(&delayReceivedCallback);

    userScheduler.addTask( taskSendMessage );
    taskSendMessage.enable();
    // 获取并打印节点ID
    RemoteControlNodeID = mesh.getNodeId();
    Serial.printf("Node ID: %u\n", RemoteControlNodeID);


    String LVGL_Arduino = "Hello LVGL! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println(LVGL_Arduino);
    Serial.println("I am ESP32_Display_Panel");

    panel = new ESP_Panel();

    /* Initialize LVGL core */
    lv_init();

    /* Initialize LVGL buffers */
    static lv_disp_draw_buf_t draw_buf;
    /* Using double buffers is more faster than single buffer */
    /* Using internal SRAM is more fast than PSRAM (Note: Memory allocated using `malloc` may be located in PSRAM.) */
    uint8_t *buf = (uint8_t *)heap_caps_calloc(1, LVGL_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    assert(buf);
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, LVGL_BUF_SIZE);

    /* Initialize the display device */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = ESP_PANEL_LCD_H_RES;
    disp_drv.ver_res = ESP_PANEL_LCD_V_RES;
    disp_drv.flush_cb = lvgl_port_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

#if ESP_PANEL_USE_LCD_TOUCH
    /* Initialize the input device */
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_port_tp_read;
    lv_indev_drv_register(&indev_drv);
#endif
    /* Initialize bus and device of panel */
    panel->init();
#if ESP_PANEL_LCD_BUS_TYPE != ESP_PANEL_BUS_TYPE_RGB
    /* Register a function to notify LVGL when the panel is ready to flush */
    /* This is useful for refreshing the screen using DMA transfers */
    panel->getLcd()->setCallback(notify_lvgl_flush_ready, &disp_drv);
#endif

    /**
     * These development boards require the use of an IO expander to configure the screen,
     * so it needs to be initialized in advance and registered with the panel for use.
     *
     */
    Serial.println("Initialize IO expander");
    /* Initialize IO expander */
    // ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    ESP_IOExpander *expander = new ESP_IOExpander_CH422G(I2C_MASTER_NUM, ESP_IO_EXPANDER_I2C_CH422G_ADDRESS_000);
    expander->init();
    expander->begin();
    expander->multiPinMode(TP_RST | LCD_BL | LCD_RST | SD_CS | USB_SEL, OUTPUT);
    expander->multiDigitalWrite(TP_RST | LCD_BL | LCD_RST | SD_CS, HIGH);

    // Turn off backlight
    // expander->digitalWrite(USB_SEL, LOW);
    expander->digitalWrite(USB_SEL, LOW);
    /* Add into panel */
    panel->addIOExpander(expander);

    /* Start panel */
    panel->begin();

    /* Create a task to run the LVGL task periodically */
    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    xTaskCreate(lvgl_port_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);

    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    lvgl_port_lock(-1);

    
    ui_init();

    /* Release the mutex */
    lvgl_port_unlock();

    Serial.println("Setup done");
}

void loop()
{
    // Serial.println("Loop");
    mesh.update();
   // digitalWrite(LED, !onFlag);

    if (Serial.available() > 0) 
    {
      receivedCommand = Serial.readStringUntil('\n');                     // 读取直到换行符
      receivedCommand.trim();
      if (receivedCommand.length() == 2 && (receivedCommand[0] == 'A' || receivedCommand[0] == 'B' || receivedCommand[0] == 'C') && (receivedCommand[1] == '0' || receivedCommand[1] == '1'))
      {    
        Serial.printf("Serial Received command: %s\n", receivedCommand.c_str());
      }    
    }
    // sleep(1);
}

String lastCommand; // 声明一个全局变量来存储上一次的 command
void sendMessage() {
  //String command;
  // 创建 JSON 格式的控制命令，其中receivedCommand为串口收到的命令
  if (!receivedCommand.isEmpty() && (receivedCommand == "A0" || receivedCommand == "A1" || 
                                      receivedCommand == "B0" || receivedCommand == "B1" || 
                                      receivedCommand == "C0" || receivedCommand == "C1")) {
    // 拼装完整的 JSON 字符串
    command = "{\"Iam\":\"RemoteControl\",\"Id\":" + String(RemoteControlNodeID) + ",\"command\":\"" + receivedCommand + "\"}";
  } else {
    // 只拼装 Id 的 JSON 字符串
    command = "{\"Iam\":\"RemoteControl\",\"Id\":" + String(RemoteControlNodeID) + "}";
  }

  // 打印调试信息
  Serial.printf("Serial Received command: %s\n", receivedCommand.c_str());
  Serial.printf("Constructed command: %s\n", command.c_str());

  // 判断 command 是否与上一次相同
  if (command == lastCommand) {
      // 如果相同，延长执行时间（例如，使用 delay 或其他逻辑）
      Serial.println("Command is the same as last time, delaying execution...");
      vTaskDelay(pdMS_TO_TICKS(1000)); // 延迟 1000 毫秒（1 秒）
  } else {
      // 如果不同，立即执行
      mesh.sendBroadcast(command);
      lastCommand = command; // 更新上一次的 command
  }

  if (calc_delay) {
    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      mesh.startDelayMeas(*node);
      node++;
    }
    calc_delay = false;
  }

  Serial.printf("Sending message: %s\n", command.c_str());  
  taskSendMessage.setInterval( random(TASK_SECOND * 1, TASK_SECOND * 5));  // between 1 and 5 seconds
}

//用于判断主机是否受到遥控器发出的命令，如果再次受到的命令一致，则遥控器将再次收到的信号比对，并清空
void receivedCallback(uint32_t from, String & msg) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
  // 判断 command 和 msg 是否相等，如果收到，则代表主机已经收到该命令，并应该执行了相关命令，则不需要再发送命令了。
  
  // {"Iam":"remotecontroler","Id":3520337297}
  // {"Iam":"remotecontroler","Id":3520337297,"command":"A1"}
  // {"Iam":"MainBoard","Id":491568245}
  if (command == msg) {
    // 检查 command 中是否包含 "command" 的 JSON 对
    if (command.indexOf("\"command\":") != -1) {
        receivedCommand = ""; // 将 receivedCommand 赋值为空
    }
  }

    // 创建一个 JSON 文档
  StaticJsonDocument<200> doc; // 根据需要调整大小

  // 解析 JSON 字符串
  DeserializationError error = deserializeJson(doc, msg);

  // 检查解析是否成功
  if (error) {
      Serial.print("Failed to parse JSON: ");
      Serial.println(error.f_str());
      return;
  }

  // 获取 Id 的值并赋值给 MainBoardID
  MainBoardID = doc["Id"];
  Serial.printf("MainBoardID: %u\n", MainBoardID);
}

// 递归函数来查找 nodeId
void searchNodeId(const JsonObject& obj, bool& foundRemoteControl, bool& foundMainBoard) {
    // 获取当前对象的 nodeId
    if (obj.containsKey("nodeId")) {
        uint32_t nodeId = obj["nodeId"];
        if (nodeId == RemoteControlNodeID) {
            foundRemoteControl = true; // 找到 RemoteControlNodeID
        }
        if (nodeId == MainBoardID) {
            foundMainBoard = true; // 找到 MainBoardID
        }
    }

    // 遍历 subs 数组
    if (obj.containsKey("subs")) {
        for (const JsonObject& sub : obj["subs"].as<JsonArray>()) {
            searchNodeId(sub, foundRemoteControl, foundMainBoard); // 递归调用
        }
    }
}

bool meshConnectionStatus = false;    // 默认状态
void newConnectionCallback(uint32_t nodeId) { 
  Serial.printf("--> remotecontroler: New Connection, nodeId = %u\n", nodeId);
  String meshSubConnection=mesh.subConnectionJson(true);
  Serial.printf("--> remotecontroler: New Connection, %s\n", meshSubConnection.c_str()); 
  /*
    一对一节点连接方式
    {
      "nodeId": 491568245,
      "subs": [
        {
          "nodeId": 3520337297
        }
      ]
    }  
    或多个节点连接方式
    {
      "nodeId": 491568245,
      "subs": [
        {
          "nodeId": 3186874889,
          "subs": [
            {
              "nodeId": 3520337297
            }
          ]
        }
      ]
    }
  */

  // 创建一个 JSON 文档
  StaticJsonDocument<300> doc; // 根据需要调整大小

  // 解析 JSON 字符串
  DeserializationError error = deserializeJson(doc, meshSubConnection);

  // 检查解析是否成功
  if (error) {
      Serial.print("Failed to parse JSON: ");
      Serial.println(error.f_str());
      return;
  }

  // 初始化找到的标志
  bool foundRemoteControl = false;
  bool foundMainBoard = false;

  // 从根对象开始搜索
  searchNodeId(doc.as<JsonObject>(), foundRemoteControl, foundMainBoard);

  // 根据找到的结果设置 meshConnectionStatus
  if (foundRemoteControl && foundMainBoard) {
      meshConnectionStatus = true; // 同时找到了
  } else {
      meshConnectionStatus = false; // 找到一个则为 false
  }

    // 打印状态
    Serial.printf("meshConnectionStatus: %s\n", meshConnectionStatus ? "true" : "false");
}


void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
 
  nodes = mesh.getNodeList();

  Serial.printf("Num nodes: %d\n", nodes.size());
  Serial.printf("Connection list:");

  SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end()) {
    Serial.printf(" %u", *node);
    node++;
  }
  Serial.println();
  calc_delay = true;
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void delayReceivedCallback(uint32_t from, int32_t delay) {
  Serial.printf("Delay to node %u is %d us\n", from, delay);
}
