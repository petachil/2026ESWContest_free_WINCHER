# 2026ESWContest_WINCHER
하이브리드 윈치 제어기/WINCHER
---
WINCHER의 하이브리드 윈치 제어기는 ESP32 환경에서 슬라이딩모드 제어와 온디바이스 머신러닝으로 윈치의 진자운동을 댐핑하는 시스템입니다.

무게추가 조건에서 제어입력 에너지와 임무시간 증가를 약 6% 내로 억제하면서도 RMSE 약 30%, 진동 에너지 약 50%를 감소하였습니다.

소개 영상: https://youtu.be/AWByFFXXPXA

---
파일 가이드
<br><br>
1. **final_esp32_winch_control_1/2_smc_flutter**
 
   platformio.ini
   
   src/
   
   main.cpp
   
   wind_model_data.h
   <br><br> 

   
Vscode PlatformIO 환경에서 Espressif ESP32 Dev Module를 빌드 후 위 파일을 덮어씁니다.

메인 ESP32로 페이로드의 각도, 사용자의 임무 지령 등 모든 정보를 종합하여 모터를 제어합니다.

1번과 2번의 코드는 통신 ID를 제외하면 완전히 똑같은 코드입니다.

시연장에서 두 제어기로 제어/비제어를 비교 시연할 예정입니다.

<br><br>
2. **mini_eps32c3_winch_payload**

   platformio.ini
   
   src/
   
   main.cpp
<br><br>
   

Vscode PlatformIO 환경에서  Seeed Studio XIAO ESP32C3를 빌드 후 위 파일을 덮어씁니다.

페이로드에 장착된 서브 EPS32C3으로 페이로드의 각도를 측정하여 메인 ESP32로 전송합니다.

<br><br>
3. **final_winch_gui_1.py**

VScode 환경에서 miniconda3로 파이썬 파일을 실행합니다. 코드 컴파일 후 요구하는 PyQt5 등을 설치해야 합니다. 

사용자 PC를 위한 그래픽사용자인터페이스 (GUI)으로 메인 ESP32로 임무지령을 보내고 실시간 텔레메트리를 수신합니다.

2개의 윈치를 동시에 제어하고 정보를 수신하며 임무 종료 후 지표분석을 수행할 수 있습니다.

<br><br>
4. **tiny_ml_1.ipynb**

VScode 환경에서 Jupyter Notebook으로 실행합니다.

윈치 실시간 동작에 관여하지 않으며, ESP32 tinyML 모델 학습을 위한 파일입니다.

<br><br>
5. **data_analyzer_final_smc_fortest5_flutter.py**

VScode 환경에서 파이썬 파일을 실행합니다. 

윈치 실시간 동작에 관여하지 않으며, 윈치의 CSV 데이터를 분석하여 그래프를 만들고 계산하는 파일입니다.

<br><br>
6. **appendix2_tinyml_environment_final.pdf**
   
   보고서 부록2 입니다. 

   tinyML 학습 환경이 담겨있습니다.

<br><br> 
7. **appendix3_test_results_final.pdf**
   
   보고서 부록3입니다.

   최종 평가에 측정한 24개의 임무의 개별 데이터와 그래프가 담겨있습니다.

<br><br> 
8. **appendix4_winch_log.zip**
   
   보고서 부록4입니다.

   최종 평가에 수집한 24개 임무의 원본 CSV 파일입니다.

---
* 본 프로젝트는 생성형 AI를 보조도구로 활용하였습니다.
