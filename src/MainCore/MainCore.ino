#ifndef ARDUINO_ARCH_SPRESENSE
#error "Board selection is wrong!!"
#endif
#ifdef SUBCORE
#error "Core selection is wrong!!"
#endif

#include <nnabla-c-runtime.h>
#include "MainRuntime_inference.h"
#include "MainRuntime_parameters.h"
#include <MP.h>
#include <Camera.h>
#include <RTC.h>
#include <OneKeySynthesizerFilter.h>
#include <SFZSink.h>

static bool doJump = false;       // ユーザーがジャンプをしたかどうか True = ジャンプをした false = ジャンプをしていない
static bool inRing = false;       // ユーザーの足がリング内に存在しているかどうか True = 入っている false = 入っていない
static bool doPlaySound = false;  // sound再生を実行しているかどうか True = sound再生中 false = sound停止中
static uint32_t soundPlayTime;    // TODO変えるかも

// サブコア制御用変数
static uint32_t snddata = 1;  //random(32767);
static uint32_t rcvdata;
static int8_t rcvid;

// サブコア制御用
const int SUBCORE = 1;  // Communication with SUBCORE2 1は音楽再生ライブラリにて利用するかも
const int USER_MSG_ID = 100;
const int TIMEOUT_MS = 1000;

// 音楽制御用
const uint8_t PLAY_CHANNEL = 1;

//画像認識AI用
const int DNN_WIDTH = 96;
const int DNN_HEIGHT = 72;
const int IN_RING = 1;  // ただのラベル 0 = リング内に入っている。リング内に入ってない...なんだけどなんかAIバグってて逆の数値になっているから1をリンクインとして判定する　

static SFZSink sink("SawLpf.sfz");
OneKeySynthesizerFilter *inst;

// Create arrays for input and output data
float input_data[NNABLART_MAINRUNTIME_INPUT0_SIZE] = { 0.0 };
float inferenceResult[NNABLART_MAINRUNTIME_OUTPUT0_SIZE] = { 0.0 };
void *context = nnablart_mainruntime_allocate_context(MainRuntime_parameters);
//nnablart_mainruntime_free_context(context);

void checkMemory(const char *additionalInfo) {
  // この処理を使うとメモリの使用量が確認できる。しかし使うとなぜか音が鳴らなくなるのでコメントアウト）
  // int usedMem, freeMem, largestFreeMem;
  // MP.GetMemoryInfo(usedMem, freeMem, largestFreeMem);
  // MPLog("Used:%4d [KB] / Free:%4d [KB] (Largest:%4d [KB]) %s\n",
  //       usedMem / 1024, freeMem / 1024, largestFreeMem / 1024, additionalInfo);
}

/* サブコア関連処理*/
void initSubCore() {
  int ret = MP.begin(SUBCORE);
  if (ret < 0) {
    Serial.println("MP.begin error = " + String(ret));
  }
  randomSeed(100);
}

void getAccelerationData() {
  Serial.println("Send: id=" + String(USER_MSG_ID) + " data=0x" + String(snddata, HEX));

  int ret = MP.Send(USER_MSG_ID, snddata, SUBCORE);
  if (ret < 0) {
    Serial.println("MP.Send error = " + String(ret));
  }

  MP.RecvTimeout(TIMEOUT_MS);

  ret = MP.Recv(&rcvid, &rcvdata, SUBCORE);
  if (ret < 0) {
    Serial.println("MP.Recv error = " + String(ret));
  }

  Serial.println("Recv: id=" + String(rcvid) + " data=0x" + String(rcvdata, HEX) + " : " + ((snddata == rcvdata) ? "Success" : "Fail"));
  Serial.println("Acceleration: 0x" + String(rcvdata, HEX));
}


/* 音楽再生処理 */
// めちゃくちゃカメラの処理と競合するのでフラグを使って排他制御をかけている（しょぼい）
// 競合するとメモリ壊れたり写真が取れなくなったり音が再生されない
// でも多分音が壊れる原因はinst->update();がコールされないことなのでタイマーで割り込み処理入れてあげればある程度解決する可能性あり
void startSound() {
  if (!doPlaySound) {
    doPlaySound = true;
    soundPlayTime = RTC.getTime().unixtime();
    printf("soundPlayTime is %d\n", soundPlayTime);
    inst->sendNoteOn(OneKeySynthesizerFilter::NOTE_ALL, DEFAULT_VELOCITY, PLAY_CHANNEL);
  }
}

// 指定時刻に応じて音楽再生時間を変更する
void stopSound(uint32_t stopTime = 2) {
  uint32_t elapsedTime = RTC.getTime().unixtime() - soundPlayTime;
  printf("time %d\n", elapsedTime);
  if (doPlaySound && elapsedTime >= stopTime /*サウンド起動からの経過時間(s)*/) {
    doPlaySound = false;
    inst->sendNoteOff(OneKeySynthesizerFilter::NOTE_ALL, DEFAULT_VELOCITY, PLAY_CHANNEL);
  }
}

/**
 * ★　この処理は必ずカメラの初期処理の前に呼び出すこと。
 *呼び出しをしない場合はカメラの初期処理となぜか競合して落ちる。
 */
void initSound() {
  inst = new OneKeySynthesizerFilter("twinkle-little-star.mid", sink);
  // setup instrument
  if (!inst->begin()) {
    Serial.println("ERROR: init error.");
    while (true) {
      delay(1000);
    }
  }
  // この処理は音楽再生を実行するが初回呼び出しをしておかないと一部楽譜データ？みたいなものがメモリに展開されない
  // この処理はカメラの初期処理の前に呼び出す必要がある。はずだけど。。。なんかカメラつけているとサウンドがバグリがちでよくわからない
  // またloop中に呼ばれると初回に少し時間がかかるためラグを防ぐためでもある。
  inst->sendNoteOn(OneKeySynthesizerFilter::NOTE_ALL, DEFAULT_VELOCITY, PLAY_CHANNEL);
  inst->sendNoteOff(OneKeySynthesizerFilter::NOTE_ALL, DEFAULT_VELOCITY, PLAY_CHANNEL);
}

/* カメラ　*/
void initCamera() {
  //bufferは持たない　pictureのタイミングは自力で制御するため
  auto came_err = theCamera.begin(0, 0, 0, CAM_IMAGE_PIX_FMT_NONE, 0);
  printf("Camera begin No = %d\n", came_err);

  came_err = theCamera.setStillPictureImageFormat(DNN_WIDTH, DNN_HEIGHT, CAM_IMAGE_PIX_FMT_RGB565);
  printf("Camera set Still Picture Image Format  No = %d\n", came_err);
}

CamImage takePictureWhenMuted() {
  CamImage camImage;
  // サウンド再生がされているときには何もしない　サウンドとカメラの処理が競合すると音が壊れるため。
  if (doPlaySound) {
    printf("サウンド流れて");
    return camImage;
  }

  printf("サウンド流れてないよ");
  camImage = theCamera.takePicture();
  return camImage;
}

void preprocessImage(const CamImage &img) {
  uint16_t *imgBuffPtr = (uint16_t *)img.getImgBuff();
  float f_max = 0.0;

  for (int n = 0; n < DNN_HEIGHT * DNN_WIDTH; ++n) {
    input_data[n] = (float)((imgBuffPtr[n] & 0x07E0) >> 5);
    if (input_data[n] > f_max) f_max = input_data[n];
  }
  /*　グレースケール変換　*/
  for (int n = 0; n < DNN_HEIGHT * DNN_WIDTH; ++n) {
    input_data[n] /= f_max;
  }
}

void runInference() {
  float *inTmp = nnablart_mainruntime_input_buffer(context, 0);
  for (int i = 0; i < NNABLART_MAINRUNTIME_INPUT0_SIZE; i++) {
    inTmp[i] = input_data[i];
  }
  int error = nnablart_mainruntime_inference(context);
  printf("error code is %d", error);

  float *outTmp = nnablart_mainruntime_output_buffer(context, 0);
  for (int i = 0; i < NNABLART_MAINRUNTIME_OUTPUT0_SIZE; i++) {
    inferenceResult[i] = outTmp[i];
    printf("output %d is %f", i, inferenceResult[i]);
  }
  return 0;
}

uint8_t getMaxIndex() {
  uint8_t index = 0;
  float max_value = inferenceResult[0];
  for (uint8_t i = 1; i < NNABLART_MAINRUNTIME_OUTPUT0_SIZE; i++) {
    if (inferenceResult[i] > max_value) {
      max_value = inferenceResult[i];
      index = i;
    }
  }
  return index;
}

bool isInRing(const CamImage &img) {
  preprocessImage(img);
  runInference();
  uint8_t index = getMaxIndex();
  return (index == IN_RING);
}

void setup() {
  Serial.begin(115200);
  printf("Initialize start\n");
  checkMemory("start ###############################");
  RTC.begin();
  checkMemory("RTC ###############################");
  initSubCore();
  checkMemory("initSubCore ###############################");
  initSound();
  checkMemory("initSound ###############################");
  initCamera();
  checkMemory("initCamera ###############################");
  checkMemory("end ###############################");
  printf("Initialize completed\n");
}

void loop() {
  // 音楽が再生済みのケースがあるので継続処理と停止処理を先に行う（音楽再生が実行されていれば何も起きない）
  inst->update();
  stopSound();

  // 平均加速度の取得
  getAccelerationData();
  // 加重平均加速度の合計値がxx以上であればユーザーがジャンプをしたと判断する
  doJump = (rcvdata >= 10) ? true : false;
  if (!doJump) {
    return;
  }

  auto img = takePictureWhenMuted();
  if (img.isAvailable()) {
    printf("takePicture %d\n", img.getImgBuffSize());
    if (isInRing(img)) {
      startSound();
    }
  }
}
