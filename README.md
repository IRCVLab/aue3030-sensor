# HYU-AUE3030, 자동차공학실험
## 0. 환경 설정

### 파이썬 가상 환경(virtual environment) 생성
```bash
$ python3.8 -m venv venv/aue3030-sensor
$ source venv/aue3030-sensor/bin/activate
```

### 가상 환경이 잘 설정되었는지 확인
결과가 다음으로 끝나야 함: `venv/aue3030-sensor/bin/python`.

```bash
$ which python
```

### 필수 패키지 설치
```bash
$ pip install -r requirements.txt
$ python -m ipykernel install --user --name venv --display-name "venv"
```

### (선택) spinnaker/pyspin 관련 추가 정보 확인: `spinnaker/README.md`.


## 1. 실습:
파일: `sync_method1_threshold.py`