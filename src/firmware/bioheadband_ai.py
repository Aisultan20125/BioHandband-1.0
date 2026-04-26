import asyncio
import io
from aiogram import Bot, Dispatcher, types, F
from aiogram.filters import Command
from aiogram.fsm.context import FSMContext
from aiogram.fsm.state import State, StatesGroup
from aiogram.utils.keyboard import ReplyKeyboardBuilder
from openai import AsyncOpenAI

TOKEN = 'Telegram-token'
OPENAI_API_KEY = 'sk-Token'

client = AsyncOpenAI(api_key=OPENAI_API_KEY)
bot = Bot(token=TOKEN)
dp = Dispatcher()

class AIChat(StatesGroup):
    waiting_for_question = State()

def main_menu():
    builder = ReplyKeyboardBuilder()
    builder.row(types.KeyboardButton(text="📊 Анализ логов (.txt)"))
    builder.row(types.KeyboardButton(text="🤖 ИИ Чат"))
    builder.row(types.KeyboardButton(text="ℹ️ О нас"))
    return builder.as_markup(resize_keyboard=True)

@dp.message(Command("start"))
async def cmd_start(message: types.Message, state: FSMContext):
    await state.clear()
    welcome = (
        "<b>INTELLIGENT INTERFACE: BIO-HEADBAND 1.0 ACTIVATED</b> 🧠\n\n"
        "Добро пожаловать в командный центр нейроаналитики. Вы подключены к системе "
        "<b>Bio-Headband AI</b> — децентрализованному ядру для обработки биометрической телеметрии.\n\n"
        "<b>ТЕКУЩИЕ ОПЕРАЦИОННЫЕ МОДУЛИ:</b>\n"
        "🔹 <b>Neural Data Ingestion (TXT):</b> Модуль глубокой дешифровки сигналов ЭЭГ. "
        "Система выявляет паттерны нейронной активности и классифицирует стадии сна в реальном времени.\n\n"
        "🔹 <b>Cognitive Consultant (AI Chat):</b> Прямой шлюз к ИИ-модели, "
        "оптимизированной под задачи биохакинга и академической производительности.\n\n"
        "<b>МИССИЯ ПРОЕКТА:</b>\n"
        "Трансформация биологического шума в математически точную стратегию успеха. "
        "Мы проектируем состояние вашей интеллектуальной неуязвимости.\n\n"
        "<i>«Дисциплина — это фундамент. Данные — это оружие».</i>\n\n"
        "<b>[STATUS: ОЖИДАНИЕ ТЕЛЕМЕТРИИ]</b>"
    )
    await message.answer(welcome, reply_markup=main_menu(), parse_mode="HTML")

@dp.message(F.text == "ℹ️ О нас")
async def about(message: types.Message, state: FSMContext):
    await state.clear()
    text = (
        "<b>PROJECT SPECIFICATION: BIO-HEADBAND 1.0</b> 🛰\n\n"
        "<b>Bio-Headband 1.0</b> — это высокотехнологичный программно-аппаратный комплекс, "
        "разработанный на стыке нейробиологии, микроэлектроники и систем искусственного интеллекта.\n\n"
        "<b>ТЕХНОЛОГИЧЕСКАЯ АРХИТЕКТУРА:</b>\n"
        "• <b>Hardware Layer:</b> Сбор данных на базе Arduino (200 Гц). Захват микровольтных колебаний мозга.\n"
        "• <b>Processing Layer:</b> Асинхронный бэкенд на Python для параллельной обработки данных.\n"
        "• <b>AI Analytical Core:</b> Интеграция с передовыми LLM для дешифровки нейронных осцилляций.\n\n"
        "👨‍💻 <b>LEAD ENGINEER & ARCHITECT:</b>\n"
        "Проект спроектирован и реализован <b>Айсұлтаном Сарсеном</b>.\n\n"
        "<i>«Моя дисциплина — это моя месть. Мой успех — моё заявление».</i>"
    )
    await message.answer(text, parse_mode="HTML")

@dp.message(F.text == "📊 Анализ логов (.txt)")
async def txt_analysis_req(message: types.Message, state: FSMContext):
    await state.clear()
    await message.answer("📡 <b>Инициализация порта загрузки.</b>\nПришлите файл .txt с данными устройства.", parse_mode="HTML")

@dp.message(F.document)
async def handle_txt_analysis(message: types.Message):
    if not message.document.file_name.lower().endswith('.txt'):
        await message.answer("⚠️ <b>Ошибка протокола.</b> Нужен формат .txt", parse_mode="HTML")
        return
    
    proc_msg = await message.answer("🛰 <b>Deep Analysis Start...</b>\nМатематическая верификация потока...", parse_mode="HTML")
    
    try:
        file_id = message.document.file_id
        file = await bot.get_file(file_id)
        binary_data = await bot.download_file(file.file_path)
        content = binary_data.read().decode('utf-8')
        
        response = await client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": (
                    "Ты — аналитическое ядро BioHeadband AI. ОТВЕЧАЙ СТРОГО НА РУССКОМ ЯЗЫКЕ. Уровень C2. "
                    "Твоя задача — эмпирическая обработка данных и ДВОЙНАЯ ВАЛИДАЦИЯ.\n"
                    "ПРАВИЛА:\n"
                    "1. ПРОВЕРКА БИОЛОГИЧЕСКИХ ЛИМИТОВ: Пульс должен быть 30-200. ЭЭГ от -200 до 200. "
                    "Если цифры вне этих рамок, объяви 'КРИТИЧЕСКАЯ ОШИБКА: СЕНСОР ОТКЛЮЧЕН ИЛИ ДАННЫЕ ИСКАЖЕНЫ'.\n"
                    "2. ВРЕМЕННОЙ ФИЛЬТР: Если строк данных очень мало (менее 10 минут реального времени), "
                    "заяви, что это [STATE: HARDWARE DIAGNOSTIC TEST], а не реальный сон.\n"
                    "3. Извлеки и покажи точные MIN и MAX значения из текста.\n"
                    "4. Используй ТОЛЬКО HTML <b> теги. Никаких Markdown (* или #).\n"
                    "5. Выводи СТРОГО в такой структуре:\n"
                    "<b>[NEURAL DATA DECODING COMPLETE]</b>\n\n"
                    "<b>DATA INTEGRITY:</b> (Напиши MIN/MAX значения и пройден ли биологический лимит).\n"
                    "<b>SESSION STATE:</b> (Тест оборудования или реальный мониторинг сна).\n"
                    "<b>COGNITIVE INSIGHT:</b> (Академический вывод уровня C2 на основе данных)."
                )},
                {"role": "user", "content": f"RAW TELEMETRY DATA:\n{content[:2500]}"}
            ]
        )
        await proc_msg.delete()
        await message.answer(f"✅ <b>REPORT GENERATED:</b>\n\n{response.choices[0].message.content}", parse_mode="HTML")
    except Exception as e:
        await message.answer(f"❌ <b>Системный сбой:</b> {str(e)}")

@dp.message(F.text == "🤖 ИИ Чат")
async def ai_chat_start(message: types.Message, state: FSMContext):
    await state.clear()
    await message.answer("🧠 <b>Нейро-консультант онлайн.</b>\nКанал связи защищен. Какой аспект оптимизации вас интересует?", parse_mode="HTML")
    await state.set_state(AIChat.waiting_for_question)

@dp.message(AIChat.waiting_for_question)
async def ai_chat_answer(message: types.Message, state: FSMContext):
    if message.text in ["📊 Анализ логов (.txt)", "ℹ️ О нас"]:
        await state.clear()
        if message.text == "📊 Анализ логов (.txt)": await txt_analysis_req(message, state)
        elif message.text == "ℹ️ О нас": await about(message, state)
        return

    await bot.send_chat_action(chat_id=message.chat.id, action="typing")
    try:
        res = await client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": (
                    "Ты — BioHeadband AI. ОТВЕЧАЙ СТРОГО НА РУССКОМ ЯЗЫКЕ. Академический стиль C2. "
                    "НИКАКИХ ПРИВЕТСТВИЙ. Отвечай сразу по делу. "
                    "Структура: [STATUS: АНАЛИЗ ЗАПРОСА] -> [ОТВЕТ] -> [КОГНИТИВНАЯ СТРАТЕГИЯ]. "
                    "Выделяй ключевые научные термины через <b></b>. Никаких Markdown символов."
                )},
                {"role": "user", "content": message.text}
            ]
        )
        await message.answer(res.choices[0].message.content, parse_mode="HTML")
    except Exception as e:
        await message.answer(f"⚠️ <b>Ошибка:</b> {str(e)}")

async def main():
    await dp.start_polling(bot)

if __name__ == "__main__":
    asyncio.run(main())
