from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from routers import chat

app = FastAPI()

origins = [
    "http://localhost:3000",  # Docusaurus frontend
    "http://127.0.0.1:3000",  # Another common local development address
    "http://127.0.0.1:8000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(chat.router)

@app.get("/")
def read_root():
    return {"Hello": "World"}
