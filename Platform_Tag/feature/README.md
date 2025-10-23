# UART LOG 모듈 사용 안내

이 디렉토리는 UART 기반의 간단한 로그 초기화 유틸을 제공합니다. SDK는 변경하지 않고, 프로젝트(API 레이어)에만 파일을 추가했습니다.

## 구성 파일
- APP_UART_LOG.c / APP_UART_LOG.h: UART 초기화/해제 함수 제공
- def_pin.h: UART 핀/버퍼/흐름제어 설정 매크로

## 의존 소스(프로젝트에 등록됨)
- SDK_nRF/components/libraries/uart/app_uart_fifo.c
- SDK_nRF/components/libraries/fifo/app_fifo.c

선택 사항: `SDK_nRF/components/libraries/uart/retarget.c`를 추가하면 `printf`를 UART로 리다이렉트할 수 있습니다.

## sdk_config 참고 설정
- `APP_UART_ENABLED = 1` (app_uart 활성화)
- `NRFX_UART_ENABLED` 또는 `NRFX_UARTE_ENABLED` 중 하나 활성화

대부분 예제 구성에서 이미 활성화되어 있습니다. 빌드 에러가 나면 해당 옵션을 확인하세요.

## 핀/설정 변경
`def_pin.h`에서 다음 매크로를 수정하세요.
- `UART_TX_PIN_NUMBER`, `UART_RX_PIN_NUMBER`
- `UART_TX_BUF_SIZE`, `UART_RX_BUF_SIZE`
- `UART_HWFC` (흐름제어)

보드 LED 핀 기본값(`LED_RED`, `LED_GREEN`)도 동일 파일에서 조정할 수 있습니다.

## 사용 방법
1) 필요 파일에서 헤더 포함
```c
#include "feature/APP_UART_LOG.h"
```

2) 초기화/해제 호출
```c
// 초기화 (예: main 초기 시점)
UART_LOG_INIT();

// ... 로그 송출 시 app_uart_put() 등을 사용 (또는 retarget.c 추가 후 printf 사용)

// 종료 시 (선택)
UART_LOG_UNINIT();
```

## 트러블슈팅
- 에러: `app_fifo.c: No such file or directory`
  - 올바른 경로는 `components/libraries/fifo/app_fifo.c` 입니다. 프로젝트에 해당 경로로 등록되어 있어야 합니다.
- `printf`가 UART로 출력되지 않음
  - `retarget.c`를 프로젝트에 추가하거나, `app_uart_put()`를 직접 사용하세요.

## 비고
- 본 모듈은 `PlatformTag.emProject`에 등록되어 있으며, 별도 인클루드 경로 추가 없이 동작합니다.

---

# APP_ADC 모듈 사용 안내

이 디렉토리는 ADC(Analog-to-Digital Converter) 기능을 제공합니다. 배터리 전압 측정을 위한 SAADC 설정과 핀 제어 함수를 포함합니다.

## 구성 파일
- APP_ADC.c / APP_ADC.h: ADC 초기화 및 배터리 전압 측정 함수 제공
- def_pin.h: ADC 핀 정의 (ADC_RDIV_TEIA, ADC_MEAS_TEIA)

## 의존 소스(프로젝트에 등록됨)
- SDK_nRF/components/libraries/saadc/nrf_drv_saadc.c
- SDK_nRF/components/libraries/gpiote/nrf_drv_gpiote.c

## sdk_config 참고 설정
- `SAADC_ENABLED = 1` (SAADC 활성화)
- `NRFX_SAADC_ENABLED = 1` (SAADC 드라이버 활성화)

## 핀/설정 변경
`def_pin.h`에서 다음 매크로를 수정하세요.
- `ADC_RDIV_TEIA`: 저항 분압기 두 번째 저항 핀
- `ADC_MEAS_TEIA`: 배터리 전압 측정 핀

## 사용 방법
1) 필요 파일에서 헤더 포함
```c
#include "feature/APP_ADC.h"
```

2) 초기화 및 사용
```c
// ADC 초기화
APP_ADC_Init();

// 배터리 전압 측정
uint8_t battery_voltage = APP_ADC_Measure_Battery_Voltage_Raw();

// 주기적 배터리 상태 확인
uint8_t voltage = APP_ADC_Check_Battery_Status(false);
```

## 주요 기능
- **APP_ADC_Init()**: SAADC 초기화 (8비트 해상도, 저전력 모드)
- **APP_ADC_Measure_Battery_Voltage_Raw()**: 배터리 전압 원시값 측정
- **APP_ADC_Check_Battery_Status()**: 주기적 배터리 상태 확인
- **핀 제어 함수들**: 측정을 위한 핀 상태 설정/복원

## 배터리 측정 원리
1. ADC_RDIV 핀을 LOW로 설정 (저항 분압기 접지)
2. ADC_MEAS 핀을 High-Z 입력으로 설정
3. SAADC로 전압 측정
4. 핀 상태 복원 (보호 및 전력 절약)

## 비고
- 본 모듈은 `PlatformTag.emProject`에 등록되어 있으며, 별도 인클루드 경로 추가 없이 동작합니다.
- 배터리 전압은 0.98 보정 계수가 적용됩니다.
- 측정 주기는 `BATTERY_MEASURE_AFTER_RR` 상수로 조정 가능합니다.


