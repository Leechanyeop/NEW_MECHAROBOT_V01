#include "open62541.h"
#include "driver/gpio.h"

// 예시: LED 제어 핀
#define LED_PIN 2

static UA_Boolean running = true;

// 서버에서 제어 명령을 받을 때 호출되는 콜백
static void writeLEDCallback(UA_Server *server,
                             const UA_NodeId *sessionId, void *sessionContext,
                             const UA_NodeId *nodeId, void *nodeContext,
                             const UA_NumericRange *range, const UA_DataValue *data) {
    if(UA_Variant_hasScalarType(&data->value, &UA_TYPES[UA_TYPES_BOOLEAN])) {
        UA_Boolean ledState = *(UA_Boolean*)data->value.data;
        gpio_set_level(LED_PIN, ledState);
    }
}

void app_main(void) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    UA_Server *server = UA_Server_new();
    UA_ServerConfig_setDefault(UA_Server_getConfig(server));

    // 센서 데이터 노드 추가
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    UA_Double temperature = 25.0;
    UA_Variant_setScalar(&attr.value, &temperature, &UA_TYPES[UA_TYPES_DOUBLE]);
    UA_NodeId tempNodeId = UA_NODEID_STRING(1, "Temperature");
    UA_QualifiedName tempName = UA_QUALIFIEDNAME(1, "Temperature");
    UA_NodeId parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
    UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
    UA_Server_addVariableNode(server, tempNodeId, parentNodeId,
                              parentReferenceNodeId, tempName,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              attr, NULL, NULL);

    // 제어 노드 추가 (서버가 값을 쓰면 ESP32가 LED 제어)
    UA_VariableAttributes ledAttr = UA_VariableAttributes_default;
    UA_Boolean ledState = false;
    UA_Variant_setScalar(&ledAttr.value, &ledState, &UA_TYPES[UA_TYPES_BOOLEAN]);
    UA_NodeId ledNodeId = UA_NODEID_STRING(1, "LED_Control");
    UA_QualifiedName ledName = UA_QUALIFIEDNAME(1, "LED_Control");
    UA_Server_addVariableNode(server, ledNodeId, parentNodeId,
                              parentReferenceNodeId, ledName,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              ledAttr, NULL, NULL);

    // LED 제어 콜백 등록
    UA_ValueCallback callback;
    callback.onRead = NULL;
    callback.onWrite = writeLEDCallback;
    UA_Server_setVariableNode_valueCallback(server, ledNodeId, callback);

    // 서버 실행 루프
    while(running) {
        // 예시: 센서 값 업데이트
        temperature += 0.1; // 실제로는 센서 읽기
        UA_Variant value;
        UA_Variant_setScalar(&value, &temperature, &UA_TYPES[UA_TYPES_DOUBLE]);
        UA_Server_writeValue(server, tempNodeId, value);

        UA_Server_run_iterate(server, true);
        vTaskDelay(5000 / portTICK_PERIOD_MS); // 5초마다 업데이트
    }

    UA_Server_delete(server);
}