/*
 * Copyright (c) 2018 ThanksMister LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed
 * under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.thanksmister.iot.esp8266.viewmodel

import android.app.Application
import android.arch.lifecycle.AndroidViewModel

import com.thanksmister.iot.esp8266.persistence.MessageDao
import com.thanksmister.iot.esp8266.util.DateUtils
import com.thanksmister.iot.esp8266.vo.Message
import io.reactivex.Completable
import io.reactivex.Flowable
import javax.inject.Inject

class MessageViewModel @Inject
constructor(application: Application, private val dataSource: MessageDao) : AndroidViewModel(application) {

    /**
     * Get the messages.
     * @return a [Flowable] that will emit every time the messages have been updated.
     */
    fun getMessages():Flowable<List<Message>> {
        return dataSource.getMessages()
                .filter {messages -> messages.isNotEmpty()}
    }

    init {
    }

}