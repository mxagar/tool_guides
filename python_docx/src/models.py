"""Pydantic models for document input JSON."""

from __future__ import annotations

from datetime import date
from decimal import Decimal
from pathlib import Path
from typing import Optional

from pydantic import BaseModel, ConfigDict, EmailStr, Field, computed_field


class Address(BaseModel):
    street: str
    postal_code: str
    city: str
    country: str = "Spain"

    def one_line(self) -> str:
        return f"{self.street}, {self.postal_code} {self.city}, {self.country}"


class CompanyInfo(BaseModel):
    name: str
    tax_id: str
    address: Address
    email: EmailStr
    phone: str
    website: Optional[str] = None
    logo_path: Optional[Path] = None


class ClientInfo(BaseModel):
    name: str
    surname: str
    tax_id: str
    address: Address
    email: EmailStr
    phone: Optional[str] = None

    @property
    def full_name(self) -> str:
        return f"{self.name} {self.surname}"


class LineItem(BaseModel):
    description: str
    unit_price: Decimal = Field(gt=0)
    quantity: Decimal = Field(gt=0)
    unit: str = "unit"

    @computed_field
    @property
    def subtotal(self) -> Decimal:
        return self.unit_price * self.quantity


class DocumentBaseData(BaseModel):
    model_config = ConfigDict(json_encoders={Decimal: str})

    company: CompanyInfo
    client: ClientInfo
    document_date: date
    iva_rate: Decimal = Field(default=Decimal("0.21"), ge=0)
    notes: Optional[str] = None
    items: list[LineItem]

    @computed_field
    @property
    def net_total(self) -> Decimal:
        return sum((item.subtotal for item in self.items), Decimal("0"))

    @computed_field
    @property
    def iva_total(self) -> Decimal:
        return self.net_total * self.iva_rate

    @computed_field
    @property
    def gross_total(self) -> Decimal:
        return self.net_total + self.iva_total


class BudgetData(DocumentBaseData):
    budget_id: str
    valid_until: date


class ReceiptData(DocumentBaseData):
    receipt_uuid: str
    budget_id: str
    payment_due_days: int = Field(gt=0)
    bank_account: str
