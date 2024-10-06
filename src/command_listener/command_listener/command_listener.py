# SPDX-FileCopyrightText: Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import argparse
import os

import riva.client
from riva.client.argparse_utils import add_asr_config_argparse_parameters, add_connection_argparse_parameters
import riva.client.audio_io

from nano_llm import NanoLLM, ChatHistory, ChatTemplates

import sys
from threading import Thread
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def parse_args() -> argparse.Namespace:
    default_device_info = riva.client.audio_io.get_default_input_device_info()
    default_device_index = None if default_device_info is None else default_device_info['index']
    parser = argparse.ArgumentParser(
        description="Streaming transcription from microphone via Riva AI Services",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--input-device", type=int, default=default_device_index, help="An input audio device to use.")
    parser.add_argument("--list-devices", action="store_true", help="List input audio device indices.")
    parser = add_asr_config_argparse_parameters(parser, profanity_filter=True)
    parser = add_connection_argparse_parameters(parser)
    parser.add_argument(
        "--sample-rate-hz",
        type=int,
        help="A number of frames per second in audio streamed from a microphone.",
        default=16000,
    )
    parser.add_argument(
        "--file-streaming-chunk",
        type=int,
        default=1600*3,
        help="A maximum number of frames in a audio chunk sent to server.",
    )

    args, unknown = parser.parse_known_args()
    return args


class CommandPublisher(Node):

    def __init__(self):
        super().__init__('command_publisher')
        self.publisher_ = self.create_publisher(String, 'command', 10)

    def publish(self, string):
        msg = String()
        msg.data = string
        self.publisher_.publish(msg)

class LLMAgent():

    def __init__(self, use_memory, llm_verbose):
        self.model_=NanoLLM.from_pretrained(
            os.environ['HUGGINGFACE_MODEL'],            # HuggingFace repo/model name, or path to HF model checkpoint
            api='mlc',                                  # supported APIs are: mlc, awq, hf
            api_token=os.environ['HUGGINGFACE_TOKEN'],  # HuggingFace API key for authenticated models ($HUGGINGFACE_TOKEN)
            quantization='q4f16_ft'                     # q4f16_ft, q4f16_1, q8f16_0 for MLC, or path to AWQ weights
        )

        self.system_prompt_="You are a robot with the following capabilities: 'shake', 'spin left', 'spin right', 'navigate'. If the action 'navigate' is selected, only the following locations are allowed: 'kitchen', 'bedroom', 'living room'. Select the action that matches the following command, if no action reflects the sentence say 'capability not supported'"

        self.use_memory_=use_memory
        self.llm_verbose_=llm_verbose
        self.chat_history_ = ChatHistory(self.model_, system_prompt=self.system_prompt_)

    def generate_response(self, prompt):
        
        if not self.use_memory_:
            self.chat_history_ = ChatHistory(self.model_, system_prompt=self.system_prompt_)

        self.chat_history_.append(role='user', msg=prompt)
        embedding, position = self.chat_history_.embed_chat()

        # generate bot reply
        reply = self.model_.generate(
            embedding, 
            streaming=True, 
            kv_cache=self.chat_history_.kv_cache,
            stop_tokens=self.chat_history_.template.stop,
            max_new_tokens=256,
        )

        msg = ''
        for token in reply:
            msg += token

        if self.llm_verbose_:
            print(f'Input: {prompt}')
            print('Bot response: ')
            print(f'{msg}\n')

        # save the inter-request KV cache 
        self.chat_history_.append('bot', reply)

        return msg.partition("Action taken: ")[2].partition("<|eot_id|>")[0]

def main() -> None:

    rclpy.init(args=None)
    command_publisher = CommandPublisher()

    spin_thread = Thread(target=rclpy.spin, args=(command_publisher,))
    spin_thread.start()

    llm_agent=LLMAgent(use_memory=False, llm_verbose=True)

    # Riva client configuration
    args = parse_args()
    if args.list_devices:
        riva.client.audio_io.list_input_devices()
        return
    auth = riva.client.Auth(args.ssl_cert, args.use_ssl, args.server, args.metadata)
    asr_service = riva.client.ASRService(auth)
    config = riva.client.StreamingRecognitionConfig(
        config=riva.client.RecognitionConfig(
            encoding=riva.client.AudioEncoding.LINEAR_PCM,
            language_code=args.language_code,
            max_alternatives=1,
            profanity_filter=args.profanity_filter,
            enable_automatic_punctuation=args.automatic_punctuation,
            verbatim_transcripts=not args.no_verbatim_transcripts,
            sample_rate_hertz=args.sample_rate_hz,
            audio_channel_count=1,
        ),
        interim_results=True,
    )
    riva.client.add_word_boosting_to_config(config, args.boosted_lm_words, args.boosted_lm_score)
    riva.client.add_endpoint_parameters_to_config(
        config,
        args.start_history,
        args.start_threshold,
        args.stop_history,
        args.stop_history_eou,
        args.stop_threshold,
        args.stop_threshold_eou
    )

    with riva.client.audio_io.MicrophoneStream(
        args.sample_rate_hz,
        args.file_streaming_chunk,
        device=args.input_device,
    ) as audio_chunk_iterator:
        responses_audio=asr_service.streaming_response_generator(
                audio_chunks=audio_chunk_iterator,
                streaming_config=config,
            )
        for audio_response in responses_audio:
            if not audio_response.results:
                continue
            for audio_result in audio_response.results:
                if not audio_result.alternatives:
                    continue
                transcript = audio_result.alternatives[0].transcript
                if audio_result.is_final:
                    msg = llm_agent.generate_response(transcript)
                    command_publisher.publish(msg)

    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

