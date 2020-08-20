package api

import (
	"bytes"
	"encoding/json"
	"io"
	"net/http"
	"os"
	"os/exec"
	"syscall"

	"github.com/Sirupsen/logrus"
	"github.com/julienschmidt/httprouter"
)

type SeverStatus struct {
	Status string `json:"status"`
}

func Status(w http.ResponseWriter, r *http.Request, params httprouter.Params) {
	status := SeverStatus{}

	writeResponse(w, status)
}

// Writes the response as a standard JSON response with StatusOK
func writeResponse(w http.ResponseWriter, m interface{}) {
	w.Header().Set("Content-Type", "application/json; charset=UTF-8")
	w.WriteHeader(http.StatusOK)
	if err := json.NewEncoder(w).Encode(m); err != nil {
		writeErrorResponse(w, http.StatusInternalServerError, "Internal Server Error")
	}
}

// Writes the error response as a Standard API JSON response with a response code
func writeErrorResponse(w http.ResponseWriter, errorCode int, errorMsg string) {
	w.Header().Set("Content-Type", "application/json; charset=UTF-8")
	w.WriteHeader(errorCode)
	json.NewEncoder(w).Encode(errorMsg)
}

func Inference(w http.ResponseWriter, r *http.Request, params httprouter.Params) {

	r.ParseMultipartForm(32 << 20)
	// 根据字段名获取表单文件
	formFile, _, err := r.FormFile("image")
	if err != nil {
		logrus.Errorf("Inference get form file failed: %s", err.Error())
		w.WriteHeader(http.StatusInternalServerError)
		return
	}
	defer formFile.Close()

	// 创建保存文件
	path := "/tmp/infer.img"

	os.Remove(path)

	destFile, err := os.Create(path)
	if err != nil {
		logrus.Errorf("Inference crate file failed: %s", err.Error())
		w.WriteHeader(http.StatusInternalServerError)
		return
	}
	defer destFile.Close()

	if _, err = formFile.Seek(0, io.SeekStart); err != nil {
		logrus.Errorf("Inference seek failed: %s", err.Error())
		w.WriteHeader(http.StatusInternalServerError)
		return
	}

	// 读取表单文件，写入保存文件
	_, err = io.Copy(destFile, formFile)
	if err != nil {
		logrus.Errorf("Inference copy file failed: %s", err.Error())
		w.WriteHeader(http.StatusInternalServerError)
		return
	}

	inferWrapper(w, r, params)
}

func prettyJson(w http.ResponseWriter, data interface{}) {
	w.Header().Set("Content-Type", "application/json")
	enc := json.NewEncoder(w)
	enc.SetIndent("", "  ")
	enc.Encode(data)
}

func inferWrapper(w http.ResponseWriter, r *http.Request, params httprouter.Params) {
	var args []string

	cmd := exec.Command("./flyinfer", args...)
	cmd.SysProcAttr = &syscall.SysProcAttr{
		Pdeathsig: syscall.SIGKILL,
	}

	jsonResult := new(bytes.Buffer)
	cmd.Stdout = jsonResult

	if err := cmd.Start(); err != nil {
		logrus.Errorf("inferWrapper cmd.Start() failed: %s", err.Error())
		w.WriteHeader(http.StatusInternalServerError)
	}

	err := cmd.Wait()
	if err != nil {
		logrus.Errorf("inferWrapper cmd.Wait() failed: %s", err.Error())
		w.WriteHeader(http.StatusInternalServerError)
	}

	prettyJson(w, jsonResult.String())
}
