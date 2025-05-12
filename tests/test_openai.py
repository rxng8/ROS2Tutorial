# %%

# export OPENAI_API_KEY="your_api_key_here"
import os
from dotenv import load_dotenv, dotenv_values

# Load environment variables from .env file
# load_dotenv()
config = dotenv_values(".env")
openai_api_key = config.get('OPENAI_API_KEY')

# %%


from openai import OpenAI
client = OpenAI(api_key=openai_api_key)

response = client.responses.create(
    # model="gpt-4.1",
    model="gpt-4o-mini",
    input="Write a one-sentence bedtime story about a unicorn."
)

print(response.output_text)

# %%

print(type(response))


# %%

print(response.output_text)

# %%






